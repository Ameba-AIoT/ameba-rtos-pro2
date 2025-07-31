#include <platform_opts.h>
#include "example_eap.h"
#include "FreeRTOS.h"
#include "task.h"
#include <platform_stdlib.h>
#include "wifi_fast_connect.h"


// get config arguments from wifi_eap_config.c
extern char *eap_target_ssid;
extern char *eap_identity;
extern char *eap_password;
extern const unsigned char *eap_ca_cert;
extern const unsigned char *eap_client_cert;
extern const unsigned char *eap_client_key;
extern char *eap_client_key_pwd;
extern int eap_ca_cert_len;
extern int eap_client_cert_len;
extern int eap_client_key_len;

void example_eap_config(void)
{
	eap_target_ssid = (char *)"WQC-Mobile&CE-24G";
	eap_identity = (char *)"qctest";
	eap_password = (char *)"realtek@2379";

	/*
		Set client cert is only used for EAP-TLS connection.
		If you are not using EAP-TLS method, no need to set eap_client_cert and eap_client_key value. (leave them to NULL value)
	*/
	/*
		eap_client_cert = \
	"-----BEGIN CERTIFICATE-----\r\n" \
	"MIIC9TCCAd0CAQIwDQYJKoZIhvcNAQEEBQAwgZMxCzAJBgNVBAYTAkZSMQ8wDQYD\r\n" \
	"VQQIEwZSYWRpdXMxEjAQBgNVBAcTCVNvbWV3aGVyZTEVMBMGA1UEChMMRXhhbXBs\r\n" \
	"ZSBJbmMuMSAwHgYJKoZIhvcNAQkBFhFhZG1pbkBleGFtcGxlLmNvbTEmMCQGA1UE\r\n" \
	"AxMdRXhhbXBsZSBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkwHhcNMTYwMzE0MTEzNjMy\r\n" \
	"WhcNMTcwMzE0MTEzNjMyWjBxMQswCQYDVQQGEwJGUjEPMA0GA1UECBMGUmFkaXVz\r\n" \
	"MRUwEwYDVQQKEwxFeGFtcGxlIEluYy4xGTAXBgNVBAMUEHVzZXJAZXhhbXBsZS5j\r\n" \
	"b20xHzAdBgkqhkiG9w0BCQEWEHVzZXJAZXhhbXBsZS5jb20wgZ8wDQYJKoZIhvcN\r\n" \
	"AQEBBQADgY0AMIGJAoGBAODvCWRRjVQnUyQS/OqHS8MA94Dc5UOtLagKTOMJayB5\r\n" \
	"3MZyreWBkNg6sDfDG6OSD9tkVzwcp8CtZNflJc3i+d+nAnPM+kJedPJN5YVO+uwc\r\n" \
	"+K+QObH7fEOq8hnFIvOtYOfnMAxQKaVIKk0EOqqQv06BDvLyxoDCZNpAn4NQ8ZkR\r\n" \
	"AgMBAAEwDQYJKoZIhvcNAQEEBQADggEBAItqpmFftRu8ugTy4fRFwpjJNUuMRe83\r\n" \
	"Pm5Dv3V/byCHHdmIy0UI+6ZiMEtYrpvz4ZPgk0BDeytYooT7/kEUb8niQ64bDLYo\r\n" \
	"NcXctCmn5fjyX2M6Z3lQXCxX0XdFiukWlR21w4HO0nx7OJjrcjdpP9Tyk/kzCFl7\r\n" \
	"pblIavkfSmFtcxzcp0IoCupkUjFkA+MftZF82eQx4bE0jjiw2KgGwnzyYAdgtFXv\r\n" \
	"Ednj3ZyOuTlOQNGJgLQxyHooEJ/Tol/8p9EO5S6eQaHgZhbGP3AZ3SWV5oA0e6eT\r\n" \
	"D5JXti/LhyZhcbbJFawGXFI96ZOpHJ0EW12Osx/21oqmMp12AotS5Vw=\r\n" \
	"-----END CERTIFICATE-----\r\n";
		eap_client_key = \
	"-----BEGIN RSA PRIVATE KEY-----\r\n" \
	"Proc-Type: 4,ENCRYPTED\r\n" \
	"DEK-Info: DES-EDE3-CBC,79675299AD6E2237\r\n" \
	"\r\n" \
	"ZYY2hv1PYEsrhYbCip98XNpS6XxbntynEEp6aO9UgWeQ4I1pNOUptPUE+yNhbA7X\r\n" \
	"59ueT3yzx5L2ObImlJ3eIEvWq+iB8DdcPqFAo3c4dgfw/wPEhmxVPKvIyDQfaEuA\r\n" \
	"kWUno6b07n5uLTpQjIXQSdMTMYjYS+yPQy7ONC/vl/Ce+RMzrQAZkp5xcNNarUpl\r\n" \
	"2J1D2t+eRih/zRrgeVXztMiW2uyIT5a0IPoeBTPkPVb00kWYzn8eT9doN/ZCyr83\r\n" \
	"mv/uXF5ZOHnSNleOn1NiCZ8Uu3SHnmGhMBBMI75OghpEezQQCmtefYvtRxzGjMVB\r\n" \
	"UoRIlbATAleUjk3bmqRxfA2QZJj/GFWc9grxEerHWrdThSQ0w+fvwKBjTmEtUO2+\r\n" \
	"stKBJQi9RKFq4naM8UhtxojHIscXCx/wKrRZHS4QJYOQYelzfhTRUuTf3Czm/iTh\r\n" \
	"MQvX7dITNlLE3SW2MjzHb2ON9qUaKVnQPk53DO1zYgoxgDbQrw6FXDNMtYVv8SYf\r\n" \
	"JJZp66jGX6e1t4ziPHVqlDi5D2nWQ2DPNHO/rsoydA7icncKsC0iVzeUm7XgesxD\r\n" \
	"QEZoQIQDVS1aRE7qJCk9S2Hfe5Gfqnrp4110YuN/4khjMW2cOCKa/Yjgjyy2QQXT\r\n" \
	"nn6dBAeSWGzRM059VzhOyls5FIfnJIisZvF3JG518SzBU/YUGHEVN1XsfDS2M9/q\r\n" \
	"VkqhJ8/vbmIddKGeYULYW+xs3LvU1hnWiOodd9tuSeg5PxAbkJsV1nW06mVkgBqA\r\n" \
	"zqqEvwvY+6+9QW4PClKNKSocvM6yC+uhRi0sOZ+ckOv7f+uuMyw5FQ==\r\n" \
	"-----END RSA PRIVATE KEY-----\r\n";
		eap_client_key_pwd = "testca";
	*/
	eap_client_cert = \
					  (const unsigned char *)"-----BEGIN CERTIFICATE-----\r\n" \
					  "MIIC/jCCAeYCAQIwDQYJKoZIhvcNAQELBQAwgZcxCzAJBgNVBAYTAlRXMQ8wDQYD\r\n" \
					  "VQQIEwZUYWlwZWkxDzANBgNVBAcTBlRhaXBlaTEaMBgGA1UEChQRcmVhbHRlay13\r\n" \
					  "cWNfY2VfQ0ExIDAeBgkqhkiG9w0BCQEWEXdxY19jZV9DQUB3cWMuY29tMSgwJgYD\r\n" \
					  "VQQDFB9XUUNfQ0UgQ0EgQ2VydGlmaWNhdGUgQXV0aG9yaXR5MB4XDTE2MDYwMTA2\r\n" \
					  "MzkwOVoXDTI2MDUzMDA2MzkwOVowdjELMAkGA1UEBhMCVFcxDzANBgNVBAgTBlRh\r\n" \
					  "aXBlaTEaMBgGA1UEChQRcmVhbHRlay13cWNfY2VfQ0ExGTAXBgNVBAMUEHVzZXJA\r\n" \
					  "ZXhhbXBsZS5vcmcxHzAdBgkqhkiG9w0BCQEWEHdxY191c2VyQHdxYy5jb20wgZ8w\r\n" \
					  "DQYJKoZIhvcNAQEBBQADgY0AMIGJAoGBAN10/7A1RnnI7R4bXav0b2oHc0c9ClZD\r\n" \
					  "B4PniPEhw/kVkNv+xM2Q4xi6nG+o5m+Fkg4eMcAnje5jpm8YH8E6CsiTfJCbgRYj\r\n" \
					  "Z2KULpDFQaQAn1DgcfIrmM2DkUo4N7OUqqlWGr6tc6VMrcBSNCD0lwUwo2Xlmh7H\r\n" \
					  "TpIVaGK/mbHxAgMBAAEwDQYJKoZIhvcNAQELBQADggEBAEOWI+SASasCI/xPalcA\r\n" \
					  "/d2JM11tX5UB7dmIxZdDWARric5sz85ztI2PI7XVq1NJIyuZLk2NS4sLfu6zKArK\r\n" \
					  "BZB41c1L5hLspSM0VFa2cTOizHVw9fL5K8JpPN0SG4c4oem9JnYRI0hGE6otGWAx\r\n" \
					  "wBO9JjcC60Xd+7sBRzZKVGGKwMj13GkgYAow+sCp+lLgAHrh1oss+J/kGwBBZtfr\r\n" \
					  "/NzFl1rUWeSQXFn9S3iGLH+AvmwY7dSbbvZOugeaodZgIszw6HTliDbEvoFTEAr6\r\n" \
					  "VI2pUVcMvpjmMpHYSPG/bQhNR9IOJYa9akkvgb7+w/pUl5Mw0SKGfXWIjTn3N00D\r\n" \
					  "Vuo=\r\n" \
					  "-----END CERTIFICATE-----\r\n";

	eap_client_key = \
					 (const unsigned char *)"-----BEGIN PRIVATE KEY-----\r\n" \
					 "MIICeAIBADANBgkqhkiG9w0BAQEFAASCAmIwggJeAgEAAoGBAN10/7A1RnnI7R4b\r\n" \
					 "Xav0b2oHc0c9ClZDB4PniPEhw/kVkNv+xM2Q4xi6nG+o5m+Fkg4eMcAnje5jpm8Y\r\n" \
					 "H8E6CsiTfJCbgRYjZ2KULpDFQaQAn1DgcfIrmM2DkUo4N7OUqqlWGr6tc6VMrcBS\r\n" \
					 "NCD0lwUwo2Xlmh7HTpIVaGK/mbHxAgMBAAECgYEAlIhIhjL1VfGBuFO6e/6yS3c7\r\n" \
					 "xmgWas0CWWIN4002V/Yy9prl/MpUxt1C11F9XQdFctqlm6/r7hxAIPsZMUxwtd9h\r\n" \
					 "ed/njQ3nDfmFZ4P6MmEdViI+Sh1/KGMkYJwcLm9WwwZQcRwvDWv1+9L4TIw5491b\r\n" \
					 "LJURKHwie0TiiUNwAFECQQDyokwg2QmBnlSxy9uE7u6PHrSnwlRGndugV49qDZL5\r\n" \
					 "LwZVj20vy+mR548XYaKKVYzffwnptQuX7Q0XLO0pMyLfAkEA6agOqEMCR7LycDEs\r\n" \
					 "tIl2x/Sg+eyOnQ7aHL0q1UPmuDMyFH0ccTDeHsmYQzVFkdhDvgKLpW7imvfOPU6w\r\n" \
					 "d+YVLwJBAMNLGtd9mgc4d5c8LI7M+js8TdCRu9+zA6oFkCuejWQAE6sebJYCHRgR\r\n" \
					 "N71sGrYZse/agxIXZSN97AFxadq1jCUCQCREDWJYZDY0tCRtvX6YB3OpqIKiENCX\r\n" \
					 "yYrEYa2QSHM2nwNHF+8JorAsohFsZ0vnwTvwsTQQLePXqo8hc4poj8kCQQCEjyzW\r\n" \
					 "fldkdaRDTk+2oLu4daPDBAIKDNbgom7s+XdSCTJdOuTRlFGo9tWzdKhSlIL7fEMd\r\n" \
					 "w4d24ytfPpq8v7hC\r\n" \
					 "-----END PRIVATE KEY-----\r\n";

	/*
		Verify server's certificate is an optional feature.
		If you want to use it please make sure ENABLE_EAP_SSL_VERIFY_SERVER in platform_opts.h is set to 1,
		and the eap_ca_cert is set correctly.
	*/
	eap_ca_cert = \
				  (const unsigned char *)"-----BEGIN CERTIFICATE-----\r\n" \
				  "MIIE8DCCA9igAwIBAgIJANqu7Mh8daj7MA0GCSqGSIb3DQEBCwUAMIGXMQswCQYD\r\n" \
				  "VQQGEwJUVzEPMA0GA1UECBMGVGFpcGVpMQ8wDQYDVQQHEwZUYWlwZWkxGjAYBgNV\r\n" \
				  "BAoUEXJlYWx0ZWstd3FjX2NlX0NBMSAwHgYJKoZIhvcNAQkBFhF3cWNfY2VfQ0FA\r\n" \
				  "d3FjLmNvbTEoMCYGA1UEAxQfV1FDX0NFIENBIENlcnRpZmljYXRlIEF1dGhvcml0\r\n" \
				  "eTAeFw0xNjA1MzEwODAxMTdaFw0yNjA1MjkwODAxMTdaMIGXMQswCQYDVQQGEwJU\r\n" \
				  "VzEPMA0GA1UECBMGVGFpcGVpMQ8wDQYDVQQHEwZUYWlwZWkxGjAYBgNVBAoUEXJl\r\n" \
				  "YWx0ZWstd3FjX2NlX0NBMSAwHgYJKoZIhvcNAQkBFhF3cWNfY2VfQ0FAd3FjLmNv\r\n" \
				  "bTEoMCYGA1UEAxQfV1FDX0NFIENBIENlcnRpZmljYXRlIEF1dGhvcml0eTCCASIw\r\n" \
				  "DQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAO68vMAVkocb/EFU3opPaPQrVajh\r\n" \
				  "vdDnsP88pOZjA3ZB4v+QwNOmdLNZ3sIcEJGgApUhc+h2562+S3dSbaf2ZuCKJRSF\r\n" \
				  "D1U3GiVYBEgm7/S66xvsWEfYhJhjoR0wJCdlZkgKypP2gjo58hEI1cg5n1HjwXOg\r\n" \
				  "ekXE6y35aHuIyRVwYXSU1jG8umXSLl5WqnuXK1Z8SCnRwkAcCcUfT/tPH3muIfLa\r\n" \
				  "xCJWeTA5IiTeeWbfDNn202G+zs8TIB3jiLNHmadsD222Za6cXkZ2Dl6P4EmCu+Od\r\n" \
				  "FV7VrWXZszkroyZ2XppVn5WnTEbOk8zO8gDCCFRO1sr8eILASMS72Q1H7hMCAwEA\r\n" \
				  "AaOCATswggE3MB0GA1UdDgQWBBQ9zduTM0CRowALE1b2vI6UB55JIzCBzAYDVR0j\r\n" \
				  "BIHEMIHBgBQ9zduTM0CRowALE1b2vI6UB55JI6GBnaSBmjCBlzELMAkGA1UEBhMC\r\n" \
				  "VFcxDzANBgNVBAgTBlRhaXBlaTEPMA0GA1UEBxMGVGFpcGVpMRowGAYDVQQKFBFy\r\n" \
				  "ZWFsdGVrLXdxY19jZV9DQTEgMB4GCSqGSIb3DQEJARYRd3FjX2NlX0NBQHdxYy5j\r\n" \
				  "b20xKDAmBgNVBAMUH1dRQ19DRSBDQSBDZXJ0aWZpY2F0ZSBBdXRob3JpdHmCCQDa\r\n" \
				  "ruzIfHWo+zAPBgNVHRMBAf8EBTADAQH/MDYGA1UdHwQvMC0wK6ApoCeGJWh0dHA6\r\n" \
				  "Ly93d3cuZXhhbXBsZS5vcmcvZXhhbXBsZV9jYS5jcmwwDQYJKoZIhvcNAQELBQAD\r\n" \
				  "ggEBADkrtzw+hjVg1wN/0HLSHQ8rEmZZplq6Fp+s7+QWObplnlT+X+eknqbzTpXl\r\n" \
				  "DPDT1PlTnKMq8xZeqHNzWPlWfDS0zu1eR+CrXSvHfI/tXTQjoGoQwQ/BoLA/ZdW/\r\n" \
				  "xLQLFnRi5L8tSphnLdJd4K5jNbzvi6FRg0qlHnmeXggWWokN5ve8BNZiNUUDsvyB\r\n" \
				  "IauxelqMuu0WCtX/0WIU/Awcavrh54lo06DMH+ddvYJ+34FO9B1qeoGrh38577A9\r\n" \
				  "vEYJbWvw2Y1GnY9h64i0aBeFy26LxV/q5HmVaxOazBNHjKu/d8uaRVsKgC91ttkZ\r\n" \
				  "RxsinFF8hui/Bgd6OzTjuswn4ec=\r\n" \
				  "-----END CERTIFICATE-----\r\n";

	eap_client_cert_len = strlen((char *)eap_client_cert) + 1;
	eap_client_key_len = strlen((char *)eap_client_key) + 1;
	eap_ca_cert_len = strlen((char *)eap_ca_cert) + 1;

}

static void example_eap_thread(void *method)
{
	extern int eap_start(char *method);
	example_eap_config();
	printf("\nExample: EAP\n");

	if (strcmp(method, (char *)"tls") == 0) {
		// tls must present client_cert, client_key
		eap_start((char *)"tls");
	} else if (strcmp(method, (char *)"peap") == 0) {
		eap_start((char *)"peap");
	} else if (strcmp(method, (char *)"ttls") == 0) {
		eap_start((char *)"ttls");
	} else {
		printf("Invalid method\n");
	}

	vTaskDelete(NULL);
}

void example_eap(char *method)
{
	/* disable fast connect for eap, need set it before wifi_on*/
	wifi_fast_connect_enable(0);

	if (xTaskCreate(example_eap_thread, ((const char *)"example_eap_thread"), 1024, method, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate failed\n", __FUNCTION__);
	}
}
