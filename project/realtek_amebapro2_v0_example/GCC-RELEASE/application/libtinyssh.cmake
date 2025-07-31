cmake_minimum_required(VERSION 3.6)

project(tinyssh)

set(tinyssh tinyssh)

list(
	APPEND tinyssh_sources
	${sdk_root}/component/network/tinyssh/tinysshd.c
	${sdk_root}/component/network/tinyssh/crypto/cleanup.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_hash_sha256.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_hash_sha512.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_kem_sntrup761.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_kem_sntrup761x25519.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_onetimeauth_poly1305.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_scalarmult_curve25519.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_sign_ed25519.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_sort_uint32.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_stream_chacha20.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_verify_16.c
	${sdk_root}/component/network/tinyssh/crypto/crypto_verify_32.c
	${sdk_root}/component/network/tinyssh/crypto/fe.c
	${sdk_root}/component/network/tinyssh/crypto/fe25519.c
	${sdk_root}/component/network/tinyssh/crypto/ge25519.c
	${sdk_root}/component/network/tinyssh/crypto/randombytes.c
	${sdk_root}/component/network/tinyssh/crypto/sc25519.c
	${sdk_root}/component/network/tinyssh/crypto/uint32_pack.c
	${sdk_root}/component/network/tinyssh/crypto/uint32_pack_big.c
	${sdk_root}/component/network/tinyssh/crypto/uint32_unpack.c
	${sdk_root}/component/network/tinyssh/crypto/uint32_unpack_big.c
	${sdk_root}/component/network/tinyssh/crypto/verify.c
	${sdk_root}/component/network/tinyssh/tinyssh/buf.c
	${sdk_root}/component/network/tinyssh/tinyssh/byte.c
	${sdk_root}/component/network/tinyssh/tinyssh/channel.c
	${sdk_root}/component/network/tinyssh/tinyssh/channel_subsystem.c
	${sdk_root}/component/network/tinyssh/tinyssh/connectioninfo.c
	${sdk_root}/component/network/tinyssh/tinyssh/die.c
	${sdk_root}/component/network/tinyssh/tinyssh/e.c
	${sdk_root}/component/network/tinyssh/tinyssh/env.c
	${sdk_root}/component/network/tinyssh/tinyssh/getln.c
	${sdk_root}/component/network/tinyssh/tinyssh/global.c
	${sdk_root}/component/network/tinyssh/tinyssh/iptostr.c
	${sdk_root}/component/network/tinyssh/tinyssh/load.c
	${sdk_root}/component/network/tinyssh/tinyssh/log.c
	${sdk_root}/component/network/tinyssh/tinyssh/main_tinysshd.c
	${sdk_root}/component/network/tinyssh/tinyssh/main_tinysshd_makekey.c
	${sdk_root}/component/network/tinyssh/tinyssh/newenv.c
	${sdk_root}/component/network/tinyssh/tinyssh/numtostr.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_auth.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_channel_open.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_channel_recv.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_channel_request.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_get.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_hello.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_kex.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_kexdh.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_put.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_recv.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_send.c
	${sdk_root}/component/network/tinyssh/tinyssh/packet_unimplemented.c
	${sdk_root}/component/network/tinyssh/tinyssh/packetparser.c
	${sdk_root}/component/network/tinyssh/tinyssh/porttostr.c
	${sdk_root}/component/network/tinyssh/tinyssh/randommod.c
	${sdk_root}/component/network/tinyssh/tinyssh/savesync.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_cipher.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_cipher_chachapoly.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_kex.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_kex_curve25519.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_kex_sntrup761x25519.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_key.c
	${sdk_root}/component/network/tinyssh/tinyssh/sshcrypto_key_ed25519.c
	${sdk_root}/component/network/tinyssh/tinyssh/str.c
	${sdk_root}/component/network/tinyssh/tinyssh/stringparser.c
	${sdk_root}/component/network/tinyssh/tinyssh/subprocess_auth.c
	${sdk_root}/component/network/tinyssh/tinyssh/subprocess_sign.c
	${sdk_root}/component/network/tinyssh/tinyssh/trymlock.c
	${sdk_root}/component/network/tinyssh/tinyssh/writeall.c
)

add_library(
	${tinyssh} STATIC
	${tinyssh_sources}
)

list(
	APPEND tinyssh_flags
	CONFIG_BUILD_RAM=1
	CONFIG_BUILD_LIB=1
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
)

target_compile_definitions(${tinyssh} PRIVATE ${tinyssh_flags} )
target_compile_options(${tinyssh} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

include(../includepath.cmake)
target_include_directories(
	${tinyssh}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
	${sdk_root}/component/network/tinyssh/tinyssh
	${sdk_root}/component/network/tinyssh/crypto
)
