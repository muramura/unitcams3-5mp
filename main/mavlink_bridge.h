#pragma once

#include "mavlink_router.h"

#define MAVLINK_UDP_PORT        MAVLINK_ROUTER_UDP_PORT
#define MAVLINK_BUF_SIZE        MAVLINK_ROUTER_BUF_SIZE

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Backward-compatible alias for initializing the router
 */
static inline esp_err_t mavlink_bridge_init(void)
{
    return mavlink_router_init();
}

#ifdef __cplusplus
}
#endif
