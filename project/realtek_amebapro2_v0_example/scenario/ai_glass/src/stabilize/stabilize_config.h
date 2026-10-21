#ifndef STABILIZE_CONFIG_H
#define STABILIZE_CONFIG_H

// ============================================================
// Fixed camera calibration values (easy to change here)
// SEI data layout: uuid, timestamp, x, y, z, fx, fy, cx, cy, k0, k1, k2, k3, ver
// ============================================================
#define CAMERA_CALIB_FX  1555.87419914f
#define CAMERA_CALIB_FY  1560.42336447f
#define CAMERA_CALIB_CX  1043.35359757f
#define CAMERA_CALIB_CY  807.52174729f
#define CAMERA_CALIB_K0  0.46590324f
#define CAMERA_CALIB_K1  0.65794636f
#define CAMERA_CALIB_K2  -5.04545799f
#define CAMERA_CALIB_K3  8.28017029f
#define CAMERA_CALIB_VER 1.00000000f
#define CAMERA_CALIB__HZ 800.00000000f

// ============================================================
// Camera calibration key/value pairs for the MP4 'udta / meta / keys'
// metadata (mobile side looks them up by name).
// handler_type = 'mdta', value encoded as UTF-8 decimal string.
// ============================================================
typedef struct {
	const char *key;    // key name, e.g. "com.realtek.eis.ldc.fx"
	float       value;  // calibration value
} camera_calib_key_value_t;

static const camera_calib_key_value_t g_camera_calib_keys[] = {
	{ "com.realtek.eis.ldc.fx", CAMERA_CALIB_FX },
	{ "com.realtek.eis.ldc.fy", CAMERA_CALIB_FY },
	{ "com.realtek.eis.ldc.cx", CAMERA_CALIB_CX },
	{ "com.realtek.eis.ldc.cy", CAMERA_CALIB_CY },
	{ "com.realtek.eis.ldc.k0", CAMERA_CALIB_K0 },
	{ "com.realtek.eis.ldc.k1", CAMERA_CALIB_K1 },
	{ "com.realtek.eis.ldc.k2", CAMERA_CALIB_K2 },
	{ "com.realtek.eis.ldc.k3", CAMERA_CALIB_K3 },
	{ "com.realtek.eis.ldc.ver", CAMERA_CALIB_VER },
	{ "com.realtek.eis.ldc.imu_rate_hz", CAMERA_CALIB__HZ },
};
#define CAMERA_CALIB_KEY_COUNT \
	(sizeof(g_camera_calib_keys) / sizeof(g_camera_calib_keys[0]))

#endif // STABILIZE_CONFIG_H