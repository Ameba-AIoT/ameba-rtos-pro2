#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif

typedef struct
#if defined(__GNUC__)
__attribute__((packed))
#endif
{
	uint8_t reserve;
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
}
ai_glass_fw_version_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

static int default_ai_glass_fw_version_get(ai_glass_fw_version_t *out_buf);

void ai_glass_set_custom_get_fw_version_func(void (*func)(ai_glass_fw_version_t *));

//Wrapper function to use custom or default fw version.
ai_glass_fw_version_t ai_glass_get_fw_version(void);