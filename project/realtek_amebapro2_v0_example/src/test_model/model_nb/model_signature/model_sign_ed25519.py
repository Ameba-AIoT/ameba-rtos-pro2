#!/usr/bin/env python
# ----------------------------
# Python package install
# ----------------------------
# Install PyNaCl: pip install pynacl
# Install PyCrypto: pip install pycryptodome
#
# ----------------------------
# Usage
# ----------------------------
# Key format: Hex string file
#
# Sign only:
# $ python3 model_sign_ed25519.py --sign-key "model-sign-key" --model "../yolov4_tiny.nb"
# $ python3 model_sign_ed25519.py --verify-key "model-verify-key" --signed-model "../yolov4_tiny.nb.sig"
#
# Enc only:
# $ python3 model_sign_ed25519.py --model "../yolov4_tiny.nb" --enc-key "model-enc-key"
# $ python3 model_sign_ed25519.py --signed-model "../yolov4_tiny.nb.enc" --enc-key "model-enc-key"
#
# Sign and Enc:
# $ python3 model_sign_ed25519.py --sign-key "model-sign-key" --model "../yolov4_tiny.nb" --enc-key "model-enc-key"
# $ python3 model_sign_ed25519.py --verify-key "model-verify-key" --signed-model "../yolov4_tiny.nb.enc.sig" --enc-key "model-enc-key"
#
# ----------------------------
# Format
# ----------------------------
# | model bin | sha256 | signature | iv |
#
import nacl.encoding
import nacl.hash
from nacl.encoding import HexEncoder
from nacl.signing import SigningKey
from nacl.signing import VerifyKey
from Crypto.Cipher import AES
import argparse
import os
import sys

def Sign(sign_key_path, model_path, enc_key_path=None):
	# load model binary
	model_bin_size = os.path.getsize(model_path)
	model_bin = open(model_path, "br").read()
	signed_enc_model_path = model_path

	# check whether need to encrypt the model header with AES-256-CBC 
	iv = None
	model_bin_enc512 = None
	if enc_key_path != None:
		enckey_hex = open(enc_key_path,"r").read()
		enckey = bytes.fromhex(enckey_hex)
		cipher = AES.new(enckey, AES.MODE_CBC)
		model_bin_enc512 = open(model_path, "br").read()[:512]  # always encrypt first 512bytes fixed header
		model_bin_enc512 = cipher.encrypt(model_bin_enc512)
		iv = cipher.iv
		print("iv (Hex):", iv.hex())
		model_bin = model_bin_enc512 + model_bin[512:model_bin_size]
		signed_enc_model_path = signed_enc_model_path + ".enc"
	else:
		iv = bytes(16)
	model_bin = model_bin + iv

	# check whether need to sign the model with ED25519
	digest = None
	sig = None
	if sign_key_path != None:
		# sha256 digest
		HASHER = nacl.hash.sha256
		digest = HASHER(model_bin, encoder=nacl.encoding.RawEncoder)
		print("model sha256 (Hex):", digest.hex())
		# load private key to sign the digest and get the model signature (DSA is EDDSA_ED25519)
		skey_hex = open(sign_key_path,"rb").read()
		signing_key = SigningKey(skey_hex, encoder=HexEncoder)
		signed = signing_key.sign(digest)
		sig = signed.signature
		print("model signature (Hex):", sig.hex())
		open("model-signature-64byte","wb").write(sig)
		signed_enc_model_path = signed_enc_model_path + ".sig"

	# pad hash, signature, iv at model end
	f = open(signed_enc_model_path, "wb")
	f.write(model_bin)
	if sign_key_path != None:
		f.write(digest)
		f.write(sig)
	if enc_key_path != None:
		f.seek(0, 0)
		f.write(model_bin_enc512)
	f.close()
	print("signed/enc model -->", signed_enc_model_path)


def Verify(verify_key_path, signed_model_path, dec_key_path=None):
	# load signed/encrypted model binary
	model_bin_size = os.path.getsize(signed_model_path)
	model_bin = open(signed_model_path, "br").read()

	# check whether need to verify the model signature with ED25519
	if verify_key_path != None:
		# load hex serialized public key to verify signature
		vkey_hex = open(verify_key_path,"rb").read()
		print("Public key (Hex):", vkey_hex)
		verifying_key = VerifyKey(vkey_hex, encoder=HexEncoder)
		# Signature check
		sig_read = model_bin[model_bin_size-64:model_bin_size]
		print("read model signature (Hex):", sig_read.hex())
		digest_read = model_bin[(model_bin_size-64-32):(model_bin_size-64)]
		try:
			verifying_key.verify(digest_read, sig_read)
			print("model signature is correct!")
		except nacl.exceptions.BadSignatureError:
			print("model signature is bad!")

		# calculate sha256 digest
		model_bin_size = model_bin_size-64-32
		model_bin_signed = model_bin[:model_bin_size]
		HASHER = nacl.hash.sha256
		digest = HASHER(model_bin_signed, encoder=nacl.encoding.RawEncoder)
		print("model sha256 (Hex):", digest.hex())
		# integrity check with sha256 digest
		if digest_read==digest:
			print("model sha256 is correct!")
		else:
			print("model sha256 is bad!")

	# check whether need to decrypt the model header with AES-256-CBC 
	iv = None
	model_bin_dec512 = None
	if dec_key_path != None:
		deckey_hex = open(dec_key_path,"r").read()
		deckey = bytes.fromhex(deckey_hex)
		iv = model_bin[model_bin_size-16:model_bin_size]  # iv appended at the end
		print("iv (Hex):", iv.hex())
		model_bin_dec512 = model_bin[:512]  # always decrypt first 512bytes fixed header
		cipher = AES.new(deckey, AES.MODE_CBC, iv)
		model_bin_dec512 = cipher.decrypt(model_bin_dec512)
		# write decypt model part for validation
		decrypted_mdoel_path = signed_model_path+".dec"
		f = open(decrypted_mdoel_path, "wb")
		f.write(model_bin[:model_bin_size-16])
		f.seek(0, 0)
		f.write(model_bin_dec512)
		f.close()
		print("save decrypted model -->", decrypted_mdoel_path)


parser = argparse.ArgumentParser(description="Sign/Enc NN model")
parser.add_argument("--sign-key", type=str, help="sign NN model with private key")
parser.add_argument("--model", type=str, help="model binary file")
parser.add_argument("--verify-key", type=str, help="verify NN model signature with public key")
parser.add_argument("--signed-model", type=str, help="signed/encrypted model binary file")
parser.add_argument("--enc-key", type=str, help="encrpyt/decrpyt model binary file with aes key")
arg = parser.parse_args()

if (arg.sign_key is not None or arg.enc_key is not None) and arg.model is not None:
	print("-> Sign model:", arg.model)
	Sign(arg.sign_key, arg.model, arg.enc_key)
if (arg.verify_key is not None or arg.enc_key is not None) and arg.signed_model is not None:
	print("-> Verify model:", arg.signed_model)
	Verify(arg.verify_key, arg.signed_model, arg.enc_key)

if len(sys.argv) == 1:
	parser.print_help()
	exit(1)
