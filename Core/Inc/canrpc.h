#ifndef CANRPC_H_INCLUDED
#define CANRPC_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CANRPC_CANFD
#define CANRPC_CANFD 1
#endif

#if CANRPC_CANFD
#define CANRPC_FRAME_MAX_LEN 64u
#define CANRPC_PUB_HDR_LEN   2u
#define CANRPC_PUB_MAX_LEN   62u
#else
#define CANRPC_FRAME_MAX_LEN 8u
#define CANRPC_PUB_HDR_LEN   1u
#define CANRPC_PUB_MAX_LEN   7u
#endif

#ifndef CANRPC_MAX_SLOTS
#define CANRPC_MAX_SLOTS      4u
#endif

#ifndef CANRPC_MAX_TXN
#define CANRPC_MAX_TXN        6u
#endif

#ifndef CANRPC_PUB_QUEUE
#define CANRPC_PUB_QUEUE      16u
#endif

#ifndef CANRPC_ACK_TIMEOUT_MS
#define CANRPC_ACK_TIMEOUT_MS 20u
#endif

#ifndef CANRPC_MAX_RETRY
#define CANRPC_MAX_RETRY      3u
#endif

#ifndef CANRPC_DONE_HOLD_MS
#define CANRPC_DONE_HOLD_MS   1000u
#endif

#define CANRPC_TYPE_CMD   1u
#define CANRPC_TYPE_ACK   2u
#define CANRPC_TYPE_DONE  3u
#define CANRPC_TYPE_PUB   4u

#define CANRPC_ID(type, node)   ((uint16_t)((((uint16_t)(type)) & 7u) << 8 | (((uint16_t)(node)) & 0xFFu)))
#define CANRPC_ID_TYPE(id)      ((uint8_t)(((id) >> 8) & 7u))
#define CANRPC_ID_NODE(id)      ((uint8_t)((id) & 0xFFu))

#define CANRPC_OK             0u
#define CANRPC_RES_BUSY       0xF0u
#define CANRPC_RES_NO_HANDLER 0xF1u
#define CANRPC_RES_REJECT     0xF2u
#define CANRPC_RES_NO_ACK     0xFDu
#define CANRPC_RES_TIMEOUT    0xFEu
#define CANRPC_RES_INVALID    0xFFu

#define CANRPC_ERR_TIMEOUT    (-1)
#define CANRPC_ERR_PARAM      (-2)

#define CANRPC_H(h)           (1u << (uint32_t)(h))

typedef struct {
    int32_t arg;
    int32_t ret;
    uint8_t result;
} canrpc_req_t;

typedef enum {
    CANRPC_H_DONE = 0,
    CANRPC_H_RUNNING,
    CANRPC_H_REJECT,
} canrpc_hstatus_t;

typedef canrpc_hstatus_t (*canrpc_handler_fn_t)(canrpc_req_t *req);

typedef struct {
    uint8_t cmd;
    canrpc_handler_fn_t start;
    canrpc_handler_fn_t poll;
} canrpc_handler_t;

typedef void (*canrpc_pub_cb_t)(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len);

void canrpc_init(uint8_t own_addr);
bool canrpc_start(uint8_t own_addr);
bool canrpc_is_started(void);
void canrpc_server_set_handlers(const canrpc_handler_t *table, uint8_t count);
void canrpc_on_rx(uint16_t id, const uint8_t *data, uint8_t len);
void canrpc_poll(void);

int canrpc_call(uint8_t node, uint8_t cmd, int32_t arg);
int canrpc_wait(uint32_t mask, uint32_t timeout_ms);
uint8_t canrpc_result(int h);
int32_t canrpc_ret(int h);
int canrpc_call_wait(uint8_t node, uint8_t cmd, int32_t arg, uint32_t timeout_ms);
int canrpc_call_wait_ret(uint8_t node, uint8_t cmd, int32_t arg, int32_t *ret, uint32_t timeout_ms);

bool canrpc_publish(uint8_t topic, const void *data, uint8_t len);
void canrpc_set_pub_handler(canrpc_pub_cb_t cb);
uint32_t canrpc_pub_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* CANRPC_H_INCLUDED */
