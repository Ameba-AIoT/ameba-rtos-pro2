//Will only provide PEM certs for now

extern const char ai_glass_ca_cert_pem[];
extern const char ai_glass_src_cert_pem[];
extern const char ai_glass_src_key_pem[];

extern const size_t ai_glass_ca_cert_pem_len;
extern const size_t ai_glass_src_cert_pem_len;
extern const size_t ai_glass_src_key_pem_len;


#define AI_GLASS_CA_CERT_PEM                                               \
    "-----BEGIN CERTIFICATE-----\r\n"                                      \
    "MIIDsTCCApmgAwIBAgIUMgS0m+sjGFQjlHdCb35H3OHXvPYwDQYJKoZIhvcNAQEL\r\n" \
    "BQAwaDELMAkGA1UEBhMCU0cxEjAQBgNVBAgMCVNpbmdhcG9yZTESMBAGA1UEBwwJ\r\n" \
    "U2luZ2Fwb3JlMRAwDgYDVQQKDAdSZWFsdGVrMQswCQYDVQQLDAJSRDESMBAGA1UE\r\n" \
    "AwwJYW5vbnltb3VzMB4XDTI1MDUyNzA2MjIxMFoXDTM1MDUyNTA2MjIxMFowaDEL\r\n" \
    "MAkGA1UEBhMCU0cxEjAQBgNVBAgMCVNpbmdhcG9yZTESMBAGA1UEBwwJU2luZ2Fw\r\n" \
    "b3JlMRAwDgYDVQQKDAdSZWFsdGVrMQswCQYDVQQLDAJSRDESMBAGA1UEAwwJYW5v\r\n" \
    "bnltb3VzMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAui7W5HK8Tldc\r\n" \
    "v+ofEPW1IDqVEbf2r6oCNIu/JkDWJk8Oe7Zo9KxHTIgaxfxFqNXHZf0904MXsOf/\r\n" \
    "juZU9H9S33TqarbSFoxiebEhhEfj52ZFKu9JGROKdUJxYCo4RYa3pFyKiQWMcTOj\r\n" \
    "3AU8IwgJ/o0f7/z4sHCPFYfzkJCARkQ1eeDhPAJRI+id8olYtObKomLxEBmAEzNs\r\n" \
    "EwJu3StERpTCnNsyy4BLAWkjxLm2J6/J4W7Wr1DWZ9bhep9hlwVC/VzIpCyzXmT2\r\n" \
    "BT4k22g5ucK3FNKHAMsmFypIb/jAguKG9QG3w0DTJy3GG5ooTtGQmm2DVeuqYBvr\r\n" \
    "D8eolwnxpwIDAQABo1MwUTAdBgNVHQ4EFgQUoU/uDGe+tNZhZxlLvgxKeLt4uSYw\r\n" \
    "HwYDVR0jBBgwFoAUoU/uDGe+tNZhZxlLvgxKeLt4uSYwDwYDVR0TAQH/BAUwAwEB\r\n" \
    "/zANBgkqhkiG9w0BAQsFAAOCAQEAfl+au7l5hhg/gmDpCMf8IWQEPDRKF7kwBbM0\r\n" \
    "Gpc15U2MT2+4pxOJzsuQcFOB9p7mVdiHtbLviUobpc2PqjZLm16/7iUk8aFMV51J\r\n" \
    "Q0uwW9kFP8EvgZk9jzydrPB2hmgNi45oTgfmsk7Aq7zSAa6sUu9jHZXNReOAO4oe\r\n" \
    "vt6BgG2+HYZoqlWdb0nu5nhxR5KvXNvInyzQP/Nv3QevBkhLpTBPIowN+iNsyJsT\r\n" \
    "lEelZW3Vf1qHWFNC8rb4qdfHPa032LSHfZ4kxw35Mg1KSfKTdolXndZ6I5AhLbLb\r\n" \
    "5yOe44WpzhfClHtEE/g5CqO/DRxTYMTI3Y7W4NTYWTT1B00F9w==\r\n"             \
    "-----END CERTIFICATE-----\r\n"

#define AI_GLASS_SRC_CERT_PEM                                              \
     "-----BEGIN CERTIFICATE-----\r\n"                                     \
    "MIIDQzCCAisCFDwm+DKo7xdxNY4gzG3RfbKYyjSHMA0GCSqGSIb3DQEBCwUAMGgx\r\n" \
    "CzAJBgNVBAYTAlNHMRIwEAYDVQQIDAlTaW5nYXBvcmUxEjAQBgNVBAcMCVNpbmdh\r\n" \
    "cG9yZTEQMA4GA1UECgwHUmVhbHRlazELMAkGA1UECwwCUkQxEjAQBgNVBAMMCWFu\r\n" \
    "b255bW91czAeFw0yNTA1MjcwNjIzMzRaFw0zNTA1MjUwNjIzMzRaMFQxCzAJBgNV\r\n" \
    "BAYTAlNHMRIwEAYDVQQIDAlTaW5nYXBvcmUxEjAQBgNVBAcMCVNpbmdhcG9yZTEQ\r\n" \
    "MA4GA1UECgwHUmVhbHRlazELMAkGA1UECwwCUkQwggEiMA0GCSqGSIb3DQEBAQUA\r\n" \
    "A4IBDwAwggEKAoIBAQC7Df+ByiqXhDtMebjJAw626nH6cC2B+c9Q6SLDoEdIU8Do\r\n" \
    "dmp7xr9EMfu3b35WcmaNYpxlBWpdQ7z8Tlg3hKsqRtCEH/qo/FBafoS7zhXd4Edh\r\n" \
    "2/X6pVDRatDZPrknyLQbujddCSp5I4VRk0eC9sP+6kfjAAHx6KgGFniSLTb8rwe6\r\n" \
    "MjjsxUjY3+alqHKBTnfeNMUemqVhY3E/YTntsfTHVoqAwBn804FFoQl7VbUYf/QK\r\n" \
    "Xv3aBPHQNnY2OihBzfT4SWGRtl+XC91VW9VZJm2csN/OK5MRRDHj6l2FOfp7srBh\r\n" \
    "NgAwpawJnrd96SZovw14UTPxgd3bqeNAzrvSnMMTAgMBAAEwDQYJKoZIhvcNAQEL\r\n" \
    "BQADggEBAHUj83THzU3YO17p7CsPnqUL2RvQU13ZxA8k56YV/zOfViPgvVPWyiy8\r\n" \
    "k7QSr3TbdNkk77DL7k64hKQLTetwEEYAu3kpNGlNmV5+SI1+tTfsNHmZoQgLK4L0\r\n" \
    "mKPODuXjWG9BX+rY+oW/pwmsXJFiO1JWYgn14U34IP0qlOeMR+4DIqLl7i5qxPQ3\r\n" \
    "Yvf25lHXwu+Sj/9yrt3NMORCkqXfQAnfAjf7Ao2ZV4OarOJy/0IjD4cQiaZWzUx8\r\n" \
    "1NiqwK9DNxazWHMDZ0s5qeQNY/Dbjj4Oga0ihcRga6XZZOQ1FcwwPxRxAw+XgQls\r\n" \
    "9QO/i6A6vr337FYkHgYeHQ0RHP6ujG8=\r\n"                                 \
    "-----END CERTIFICATE-----\r\n"


#define AI_GLASS_SRC_KEY_PEM                                               \
     "-----BEGIN PRIVATE KEY-----\r\n"                                     \
    "MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQC7Df+ByiqXhDtM\r\n" \
    "ebjJAw626nH6cC2B+c9Q6SLDoEdIU8Dodmp7xr9EMfu3b35WcmaNYpxlBWpdQ7z8\r\n" \
    "Tlg3hKsqRtCEH/qo/FBafoS7zhXd4Edh2/X6pVDRatDZPrknyLQbujddCSp5I4VR\r\n" \
    "k0eC9sP+6kfjAAHx6KgGFniSLTb8rwe6MjjsxUjY3+alqHKBTnfeNMUemqVhY3E/\r\n" \
    "YTntsfTHVoqAwBn804FFoQl7VbUYf/QKXv3aBPHQNnY2OihBzfT4SWGRtl+XC91V\r\n" \
    "W9VZJm2csN/OK5MRRDHj6l2FOfp7srBhNgAwpawJnrd96SZovw14UTPxgd3bqeNA\r\n" \
    "zrvSnMMTAgMBAAECggEAURiPhricYJYuk0hOaa+OqJTaF2adBDXGwOBidwe8/8l3\r\n" \
    "4BC+t60B7VHyPKP01QvCFFgFT/LaG1crzwM0UdWN08Vhz3aB2NOTDri9WSLO7iO3\r\n" \
    "LuELMsCLWk+B/j2oTmxAUakZKZe4t05EFYrRvSC49oKh6fNt5HAmsIT7fvxlU4Tc\r\n" \
    "3fAjuvUgcXdQ3Kkbmh3tyimBsNJb8ff4GuHvm0lcSlZzXiYLYSQ+38r9KOQFTIAU\r\n" \
    "iajaU+HlLxjOdT5EWOxJ9gXF14nVsEfsrfISj0Pwqa97WRspDHbEQIE1kzf2zpfa\r\n" \
    "ZS9Q+0PTuE4+bmZdjaorjdovgYUQzriHCSK2bPdWCQKBgQDkO2zjmBRvgxSeY/20\r\n" \
    "wJVfhTxjAHcgO5cMavpbAYeOoFm0MPA0FLbPUxbf2l3hbT2IOqs6FbhWAMG++rgA\r\n" \
    "9h5ijXgP1q+N8K41iyNseC5ENtBC/igF1rinCAJyAMycQtlxnDf8QxPmQqWfTIpG\r\n" \
    "mxN4Lw2qvqVLoWqbpj+o06nXLQKBgQDR0Ay0RuOLXdnkuUP0BtxcZiGRskVkAY1X\r\n" \
    "bJkEDPG0ApSqqYCmlUA4oxBnBhODQmg6AbHopZ5K1oLEK5lrwGnDiWird4uBr3gp\r\n" \
    "wDHKtLU8gu7m7FYr7vrvY8NSILx1DqrDTkCpF+ZinxYU+HKPuPxh65JTJoyBeI6b\r\n" \
    "B65HlwVrPwKBgQC/IEylo5upbpn7sjyp+4Sbg1X4ilE4Ou7ZRVT2lSdR91JnpXvi\r\n" \
    "AV697BUBzTpFJ1gaFxeBAuNlkiitqAQjOfhkC5h5mw1UzjL1P9mgYlxMX6K0F2ao\r\n" \
    "zRHBPuHdWnH+gbTagToZFIs7jqBn8I7zZbY+NRk63YBK/5fpVKWG2gom2QKBgBxi\r\n" \
    "SvWwiWP3RFz++0RuES2m22+8cErBMv/avzCfF6vElZwo5jCjDFcdKEDnv4gIWdVP\r\n" \
    "GWRh03JIZtRnGZBMLYK4eiKIP6VBub3cNGA55jLTe8JdwqKa5/OuyWO47gXgABX/\r\n" \
    "5ht6Ej7RSsl3evgajHoqxlbdZjC1wIUUmu0wbxk9AoGBALXygC/02qEEmtsW+kRN\r\n" \
    "zAGrJi4SquxMt4hdDRApcp63fl/JTcJeBBMNaRf/XExQ5I/cIs7YuMHQZqq/kmbG\r\n" \
    "3hhp8DDGgtNzkWjHRb/pkp3Gps+env/XrOaCMkwqJT2ghqYGgL486Zoe+m3E5yvB\r\n" \
    "cGW5GoXoiTW+tZdhui6iPEcA\r\n"                                         \
    "-----END PRIVATE KEY-----\r\n"


const char ai_glass_ca_cert_pem[]   = AI_GLASS_CA_CERT_PEM;
const char ai_glass_src_cert_pem[]  = AI_GLASS_SRC_CERT_PEM;
const char ai_glass_src_key_pem[]   = AI_GLASS_SRC_KEY_PEM;

const size_t ai_glass_ca_cert_pem_len =
	sizeof(ai_glass_ca_cert_pem);

const size_t ai_glass_src_cert_pem_len =
	sizeof(ai_glass_src_cert_pem);

const size_t ai_glass_src_key_pem_len =
	sizeof(ai_glass_src_key_pem);









