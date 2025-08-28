#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "ai_glass_version.h"
#include "ai_glass_dbg.h"

#define AI_GLASS_GET_FW_VERSION_OKAY 0
#define AI_GLASS_GET_FW_VERSION_INVALID_BUFFER -1

// VERSION_RESERVE, VERSION_MAJOR, VERSION_MINOR, VERSION PATCH is originally set from scenario.cmake.
// Default ai_glass_fw_version is retrieved from the scenario.cmake. Users will have to define the firmware version by using
// flag -DVERSION = {VERSION_RESERVE}.{VERSION_MAJOR}.{VERSION_MINOR}.{VERSION_PATCH}

//Please note that the version set is different from the usual method of setting firmware version. This version is to be maintained
//by users. Our application notes provide another way that have a 32 bytes of firmware setting in amebapro2_firmware_ntz.json.

static ai_glass_fw_version_t current_version = {

	.reserve = (uint8_t) VERSION_RESERVE,

	.major = (uint8_t) VERSION_MAJOR,

	.minor = (uint8_t) VERSION_MINOR,

	.patch = (uint8_t) VERSION_PATCH,

};


//Default function
static int default_ai_glass_fw_version_get(ai_glass_fw_version_t *out_buf)
{
	if (out_buf) {
		*out_buf = current_version;
		return AI_GLASS_GET_FW_VERSION_OKAY;
	} else {
		AI_GLASS_ERR("Invalid Firmware Buffer\r\n");
		return AI_GLASS_GET_FW_VERSION_INVALID_BUFFER;
	}
}

//Users can set custom get_fw_version function through the callback function.
static void (*ai_glass_custom_get_fw_func)(ai_glass_fw_version_t *) = NULL;
void ai_glass_set_custom_get_fw_version_func(void (*func)(ai_glass_fw_version_t *))
{
	ai_glass_custom_get_fw_func = func;
}

ai_glass_fw_version_t ai_glass_get_fw_version(void)
{
	ai_glass_fw_version_t fw_version;
	if (ai_glass_custom_get_fw_func) {
		ai_glass_custom_get_fw_func(&fw_version);
	} else {
		//Default function
		default_ai_glass_fw_version_get(&fw_version);
		AI_GLASS_INFO("Firmware Version: %d.%d.%d.%d\r\n", fw_version.reserve, fw_version.major, fw_version.minor, fw_version.patch);
	}
	return fw_version;
}