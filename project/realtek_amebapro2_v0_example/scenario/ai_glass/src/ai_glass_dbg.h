#ifndef __AI_GLASS_DBG_H__
#define __AI_GLASS_DBG_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
// Definition of the log level
#define LOG_LEVEL_ERROR         5
#define LOG_LEVEL_WARNING       4
#define LOG_LEVEL_MESSAGE       3
#define LOG_LEVEL_INFO          2

#define WLAN_SCEN_LOG_LEVEL     LOG_LEVEL_WARNING
#define FILE_SYS_LOG_LEVEL      LOG_LEVEL_WARNING
#define AI_GLASS_LOG_LEVEL      LOG_LEVEL_INFO

#define WLAN_SCEN_ERR(format, ...) \
    do { \
        if (LOG_LEVEL_ERROR >= WLAN_SCEN_LOG_LEVEL) \
            printf("[WLAN SCEN ERR] " format, ##__VA_ARGS__); \
    } while(0)

#define WLAN_SCEN_WARN(format, ...) \
    do { \
        if (LOG_LEVEL_WARNING >= WLAN_SCEN_LOG_LEVEL) \
            printf("[WLAN SCEN WARN] " format, ##__VA_ARGS__); \
    } while(0)

#define WLAN_SCEN_MSG(format, ...) \
    do { \
        if (LOG_LEVEL_MESSAGE >= WLAN_SCEN_LOG_LEVEL) \
            printf("[WLAN SCEN MSG] " format, ##__VA_ARGS__); \
    } while(0)

#define WLAN_SCEN_INFO(format, ...) \
    do { \
        if (LOG_LEVEL_INFO >= WLAN_SCEN_LOG_LEVEL) \
            printf("[WLAN SCEN INFO] " format, ##__VA_ARGS__); \
    } while(0)

#define FILE_SYS_ERR(format, ...) \
    do { \
        if (LOG_LEVEL_ERROR >= FILE_SYS_LOG_LEVEL) \
            printf("[FILE SYS ERR] " format, ##__VA_ARGS__); \
    } while(0)

#define FILE_SYS_WARN(format, ...) \
    do { \
        if (LOG_LEVEL_WARNING >= FILE_SYS_LOG_LEVEL) \
            printf("[FILE SYS WARN] " format, ##__VA_ARGS__); \
    } while(0)

#define FILE_SYS_MSG(format, ...) \
    do { \
        if (LOG_LEVEL_MESSAGE >= FILE_SYS_LOG_LEVEL) \
            printf("[FILE SYS MSG] " format, ##__VA_ARGS__); \
    } while(0)

#define FILE_SYS_INFO(format, ...) \
    do { \
        if (LOG_LEVEL_INFO >= FILE_SYS_LOG_LEVEL) \
            printf("[FILE SYS INFO] " format, ##__VA_ARGS__); \
    } while(0)

#define AI_GLASS_ERR(format, ...) \
    do { \
        if (LOG_LEVEL_ERROR >= AI_GLASS_LOG_LEVEL) \
            printf("[AI GLASS ERR] " format, ##__VA_ARGS__); \
    } while(0)

#define AI_GLASS_WARN(format, ...) \
    do { \
        if (LOG_LEVEL_WARNING >= AI_GLASS_LOG_LEVEL) \
            printf("[AI GLASS WARN] " format, ##__VA_ARGS__); \
    } while(0)

#define AI_GLASS_MSG(format, ...) \
    do { \
        if (LOG_LEVEL_MESSAGE >= AI_GLASS_LOG_LEVEL) \
            printf("[AI GLASS MSG] " format, ##__VA_ARGS__); \
    } while(0)

#define AI_GLASS_INFO(format, ...) \
    do { \
        if (LOG_LEVEL_INFO >= AI_GLASS_LOG_LEVEL) \
            printf("[AI GLASS INFO] " format, ##__VA_ARGS__); \
    } while(0)

#endif //#ifndef __AI_GLASS_DBG_H__
