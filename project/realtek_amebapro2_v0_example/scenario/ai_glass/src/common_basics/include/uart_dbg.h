#ifndef __UART_DBG_H__
#define __UART_DBG_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
// UART CMD Service Debug
#define	_COMMON_DBG_UART_SRV_       0x00000001
#define	_COMMON_DBG_UART_CMD_       0x00000002
#define	_COMMON_DBG_SLIDING_WIN_    0x00000004

extern uint32_t ConfigDebugCommonErr;
extern uint32_t ConfigDebugCommonWarn;
extern uint32_t ConfigDebugCommonMsg;
extern uint32_t ConfigDebugCommonInfo;

#define DBG_COMMON_ERR_ON(x)        (ConfigDebugCommonErr |= (x))
#define DBG_COMMON_WARN_ON(x)       (ConfigDebugCommonWarn |= (x))
#define DBG_COMMON_MSG_ON(x)        (ConfigDebugCommonMsg |= (x))
#define DBG_COMMON_INFO_ON(x)       (ConfigDebugCommonInfo |= (x))

#define DBG_COMMON_ERR_OFF(x)       (ConfigDebugCommonErr &= ~(x))
#define DBG_COMMON_WARN_OFF(x)      (ConfigDebugCommonWarn &= ~(x))
#define DBG_COMMON_MSG_OFF(x)       (ConfigDebugCommonMsg &= ~(x))
#define DBG_COMMON_INFO_OFF(x)      (ConfigDebugCommonInfo &= ~(x))

#define UART_SRV_ERR(format, ...) \
    do { \
        if (ConfigDebugCommonErr & _COMMON_DBG_UART_SRV_) \
            printf("[UART SRV ERR] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_SRV_WARN(format, ...) \
    do { \
        if (ConfigDebugCommonWarn & _COMMON_DBG_UART_SRV_) \
            printf("[UART SRV WARN] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_SRV_MSG(format, ...) \
    do { \
        if (ConfigDebugCommonMsg & _COMMON_DBG_UART_SRV_) \
            printf("[UART SRV MSG] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_SRV_INFO(format, ...) \
    do { \
        if (ConfigDebugCommonInfo & _COMMON_DBG_UART_SRV_) \
            printf("[UART SRV INFO] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_CMD_ERR(format, ...) \
    do { \
        if (ConfigDebugCommonErr & _COMMON_DBG_UART_CMD_) \
            printf("[UART CMD ERR] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_CMD_WARN(format, ...) \
    do { \
        if (ConfigDebugCommonWarn & _COMMON_DBG_UART_CMD_) \
            printf("[UART CMD WARN] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_CMD_MSG(format, ...) \
    do { \
        if (ConfigDebugCommonMsg & _COMMON_DBG_UART_CMD_) \
            printf("[UART CMD MSG] " format, ##__VA_ARGS__); \
    } while(0)

#define UART_CMD_INFO(format, ...) \
    do { \
        if (ConfigDebugCommonInfo & _COMMON_DBG_UART_CMD_) \
            printf("[UART CMD INFO] " format, ##__VA_ARGS__); \
    } while(0)

#define SLIDING_WIN_ERR(format, ...) \
    do { \
        if (ConfigDebugCommonErr & _COMMON_DBG_SLIDING_WIN_) \
            printf("[SLIDING WIN ERR] " format, ##__VA_ARGS__); \
    } while(0)

#define SLIDING_WIN_WARN(format, ...) \
    do { \
        if (ConfigDebugCommonWarn & _COMMON_DBG_SLIDING_WIN_) \
            printf("[SLIDING WIN WARN] " format, ##__VA_ARGS__); \
    } while(0)

#define SLIDING_WIN_MSG(format, ...) \
    do { \
        if (ConfigDebugCommonMsg & _COMMON_DBG_SLIDING_WIN_) \
            printf("[SLIDING WIN MSG] " format, ##__VA_ARGS__); \
    } while(0)

#define SLIDING_WIN_INFO(format, ...) \
    do { \
        if (ConfigDebugCommonInfo & _COMMON_DBG_SLIDING_WIN_) \
            printf("[SLIDING WIN INFO] " format, ##__VA_ARGS__); \
    } while(0)

#endif //#ifndef __UART_DBG_H__
