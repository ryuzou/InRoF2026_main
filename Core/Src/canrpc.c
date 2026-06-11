#include "canrpc.h"

#include <string.h>
#include "board.h"

#if ((CANRPC_PUB_QUEUE & (CANRPC_PUB_QUEUE - 1u)) != 0u)
#error "CANRPC_PUB_QUEUE must be a power of two"
#endif

#if (CANRPC_MAX_SLOTS > 31u)
#error "CANRPC_MAX_SLOTS must be <= 31"
#endif

typedef enum {
    SLOT_FREE = 0,
    SLOT_WAIT_ACK,
    SLOT_WAIT_DONE,
    SLOT_DONE,
    SLOT_ERROR,
} slot_state_t;

typedef struct {
    slot_state_t state;
    uint8_t node;
    uint8_t seq;
    uint8_t cmd;
    uint8_t retry;
    uint8_t result;
    int32_t arg;
    int32_t ret;
    uint32_t t0_ms;
} slot_t;

typedef enum {
    TXN_FREE = 0,
    TXN_PENDING,
    TXN_RUNNING,
    TXN_COMPLETED,
} txn_state_t;

typedef struct {
    txn_state_t state;
    uint8_t seq;
    uint8_t cmd;
    bool done_sent;
    uint32_t done_tick_ms;
    canrpc_req_t req;
    const canrpc_handler_t *handler;
} txn_t;

typedef struct {
    uint8_t node;
    uint8_t topic;
    uint8_t len;
    uint8_t data[CANRPC_PUB_MAX_LEN];
} pub_msg_t;

static uint8_t g_own_addr;
static bool g_started;
static uint8_t g_seq;

static slot_t g_slot[CANRPC_MAX_SLOTS];
static txn_t g_txn[CANRPC_MAX_TXN];

static const canrpc_handler_t *g_handlers;
static uint8_t g_handler_count;

static volatile uint8_t g_pub_head;
static volatile uint8_t g_pub_tail;
static pub_msg_t g_pubq[CANRPC_PUB_QUEUE];
static uint32_t g_pub_drop;
static canrpc_pub_cb_t g_pub_cb;

static void put_i32_le_local(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)((u >> 8) & 0xFFu);
    p[2] = (uint8_t)((u >> 16) & 0xFFu);
    p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static int32_t get_i32_le_local(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                    ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) |
                    ((uint32_t)p[3] << 24));
}

static bool tx_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (!g_started) {
        return false;
    }
    return board_canrpc_tx(id, data, len);
}

static bool tx_cmd(uint8_t node, uint8_t seq, uint8_t cmd, int32_t arg)
{
    uint8_t p[6];
    p[0] = seq;
    p[1] = cmd;
    put_i32_le_local(&p[2], arg);
    return tx_frame(CANRPC_ID(CANRPC_TYPE_CMD, node), p, sizeof(p));
}

static bool tx_ack(uint8_t seq)
{
    uint8_t p[1] = { seq };
    return tx_frame(CANRPC_ID(CANRPC_TYPE_ACK, g_own_addr), p, sizeof(p));
}

static bool tx_done(uint8_t seq, uint8_t result, int32_t ret)
{
    uint8_t p[6];
    p[0] = seq;
    p[1] = result;
    put_i32_le_local(&p[2], ret);
    return tx_frame(CANRPC_ID(CANRPC_TYPE_DONE, g_own_addr), p, sizeof(p));
}

void canrpc_init(uint8_t own_addr)
{
    g_own_addr = own_addr;
    g_started = false;
    g_seq = 0;

    memset(g_slot, 0, sizeof(g_slot));
    memset(g_txn, 0, sizeof(g_txn));

    g_handlers = NULL;
    g_handler_count = 0;
    g_pub_head = 0;
    g_pub_tail = 0;
    g_pub_drop = 0;
    g_pub_cb = NULL;
}

bool canrpc_start(uint8_t own_addr)
{
    g_own_addr = own_addr;
    g_started = board_canrpc_start(own_addr);
    return g_started;
}

bool canrpc_is_started(void)
{
    return g_started;
}

void canrpc_server_set_handlers(const canrpc_handler_t *table, uint8_t count)
{
    g_handlers = table;
    g_handler_count = count;
}

static void client_on_rx(uint8_t type, uint8_t node, const uint8_t *p, uint8_t len)
{
    if (len < 1u) {
        return;
    }

    uint8_t seq = p[0];
    for (uint8_t i = 0; i < CANRPC_MAX_SLOTS; i++) {
        slot_t *s = &g_slot[i];
        if (s->node != node || s->seq != seq) {
            continue;
        }
        if (s->state != SLOT_WAIT_ACK && s->state != SLOT_WAIT_DONE) {
            continue;
        }

        if (type == CANRPC_TYPE_ACK) {
            if (s->state == SLOT_WAIT_ACK) {
                s->state = SLOT_WAIT_DONE;
            }
        } else {
            if (len < 2u) {
                return;
            }
            s->result = p[1];
            s->ret = (len >= 6u) ? get_i32_le_local(&p[2]) : 0;
            s->state = SLOT_DONE;
        }
        return;
    }
}

static void server_on_rx(const uint8_t *p, uint8_t len)
{
    if (len < 6u) {
        return;
    }

    uint8_t seq = p[0];
    for (uint8_t i = 0; i < CANRPC_MAX_TXN; i++) {
        txn_t *t = &g_txn[i];
        if (t->state == TXN_FREE || t->seq != seq) {
            continue;
        }

        if (t->state == TXN_COMPLETED) {
            t->done_sent = tx_done(t->seq, t->req.result, t->req.ret);
        } else {
            (void)tx_ack(seq);
        }
        return;
    }

    txn_t *free_txn = NULL;
    for (uint8_t i = 0; i < CANRPC_MAX_TXN; i++) {
        if (g_txn[i].state == TXN_FREE) {
            free_txn = &g_txn[i];
            break;
        }
    }

    if (free_txn == NULL) {
        (void)tx_done(seq, CANRPC_RES_BUSY, 0);
        return;
    }

    free_txn->seq = seq;
    free_txn->cmd = p[1];
    free_txn->done_sent = false;
    free_txn->done_tick_ms = 0;
    free_txn->req.arg = get_i32_le_local(&p[2]);
    free_txn->req.ret = 0;
    free_txn->req.result = CANRPC_OK;
    free_txn->handler = NULL;
    free_txn->state = TXN_PENDING;

    (void)tx_ack(seq);
}

static void pub_on_rx(uint8_t node, const uint8_t *p, uint8_t len)
{
    if (g_pub_cb == NULL) {
        return;
    }

#if CANRPC_CANFD
    if (len < 2u) {
        return;
    }
    uint8_t topic = p[0];
    uint8_t actual_len = p[1];
    if (actual_len > CANRPC_PUB_MAX_LEN) {
        return;
    }
    if ((uint8_t)(actual_len + 2u) > len) {
        return;
    }
#else
    if (len < 1u) {
        return;
    }
    uint8_t topic = p[0];
    uint8_t actual_len = (uint8_t)(len - 1u);
#endif

    uint8_t head = g_pub_head;
    uint8_t next = (uint8_t)((head + 1u) & (CANRPC_PUB_QUEUE - 1u));
    if (next == g_pub_tail) {
        g_pub_drop++;
        return;
    }

    pub_msg_t *m = &g_pubq[head];
    m->node = node;
    m->topic = topic;
    m->len = actual_len;
#if CANRPC_CANFD
    if (actual_len > 0u) {
        memcpy(m->data, &p[2], actual_len);
    }
#else
    if (actual_len > 0u) {
        memcpy(m->data, &p[1], actual_len);
    }
#endif
    g_pub_head = next;
}

void canrpc_on_rx(uint16_t id, const uint8_t *data, uint8_t len)
{
    uint8_t type = CANRPC_ID_TYPE(id);
    uint8_t node = CANRPC_ID_NODE(id);

    switch (type) {
    case CANRPC_TYPE_CMD:
        if (node == g_own_addr) {
            server_on_rx(data, len);
        }
        break;
    case CANRPC_TYPE_ACK:
    case CANRPC_TYPE_DONE:
        client_on_rx(type, node, data, len);
        break;
    case CANRPC_TYPE_PUB:
        if (node != g_own_addr) {
            pub_on_rx(node, data, len);
        }
        break;
    default:
        break;
    }
}

static const canrpc_handler_t *find_handler(uint8_t cmd)
{
    for (uint8_t i = 0; i < g_handler_count; i++) {
        if (g_handlers[i].cmd == cmd) {
            return &g_handlers[i];
        }
    }
    return NULL;
}

static void complete_txn(txn_t *t, uint32_t now_ms)
{
    t->state = TXN_COMPLETED;
    t->done_tick_ms = now_ms;
    t->done_sent = tx_done(t->seq, t->req.result, t->req.ret);
}

static void server_poll(void)
{
    uint32_t now = board_millis();

    for (uint8_t i = 0; i < CANRPC_MAX_TXN; i++) {
        txn_t *t = &g_txn[i];

        switch (t->state) {
        case TXN_PENDING: {
            t->handler = find_handler(t->cmd);
            if (t->handler == NULL || t->handler->start == NULL) {
                t->req.result = CANRPC_RES_NO_HANDLER;
                complete_txn(t, now);
                break;
            }

            canrpc_hstatus_t st = t->handler->start(&t->req);
            if (st == CANRPC_H_RUNNING) {
                t->state = TXN_RUNNING;
            } else {
                if (st == CANRPC_H_REJECT && t->req.result == CANRPC_OK) {
                    t->req.result = CANRPC_RES_REJECT;
                }
                complete_txn(t, now);
            }
            break;
        }
        case TXN_RUNNING: {
            if (t->handler == NULL || t->handler->poll == NULL) {
                t->req.result = CANRPC_RES_REJECT;
                complete_txn(t, now);
                break;
            }

            canrpc_hstatus_t st = t->handler->poll(&t->req);
            if (st != CANRPC_H_RUNNING) {
                if (st == CANRPC_H_REJECT && t->req.result == CANRPC_OK) {
                    t->req.result = CANRPC_RES_REJECT;
                }
                complete_txn(t, now);
            }
            break;
        }
        case TXN_COMPLETED:
            if (!t->done_sent) {
                t->done_sent = tx_done(t->seq, t->req.result, t->req.ret);
            }
            if ((uint32_t)(now - t->done_tick_ms) > CANRPC_DONE_HOLD_MS) {
                t->state = TXN_FREE;
            }
            break;
        default:
            break;
        }
    }
}

static void client_poll(void)
{
    uint32_t now = board_millis();

    for (uint8_t i = 0; i < CANRPC_MAX_SLOTS; i++) {
        slot_t *s = &g_slot[i];
        if (s->state != SLOT_WAIT_ACK) {
            continue;
        }

        if ((uint32_t)(now - s->t0_ms) <= CANRPC_ACK_TIMEOUT_MS) {
            continue;
        }

        if (s->retry >= CANRPC_MAX_RETRY) {
            s->result = CANRPC_RES_NO_ACK;
            s->state = SLOT_ERROR;
        } else {
            s->retry++;
            (void)tx_cmd(s->node, s->seq, s->cmd, s->arg);
            s->t0_ms = now;
        }
    }
}

static void pub_poll(void)
{
    while (g_pub_tail != g_pub_head) {
        pub_msg_t *m = &g_pubq[g_pub_tail];
        if (g_pub_cb != NULL) {
            g_pub_cb(m->node, m->topic, m->data, m->len);
        }
        g_pub_tail = (uint8_t)((g_pub_tail + 1u) & (CANRPC_PUB_QUEUE - 1u));
    }
}

void canrpc_poll(void)
{
    server_poll();
    client_poll();
    pub_poll();
}

int canrpc_call(uint8_t node, uint8_t cmd, int32_t arg)
{
    int h = -1;
    for (uint8_t i = 0; i < CANRPC_MAX_SLOTS; i++) {
        if (g_slot[i].state == SLOT_FREE) {
            h = (int)i;
            break;
        }
    }

    if (h < 0) {
        return -1;
    }

    if (++g_seq == 0u) {
        g_seq = 1u;
    }

    slot_t *s = &g_slot[h];
    s->state = SLOT_WAIT_ACK;
    s->node = node;
    s->seq = g_seq;
    s->cmd = cmd;
    s->retry = 0;
    s->result = CANRPC_OK;
    s->arg = arg;
    s->ret = 0;
    s->t0_ms = board_millis();

    (void)tx_cmd(node, s->seq, cmd, arg);
    return h;
}

int canrpc_wait(uint32_t mask, uint32_t timeout_ms)
{
    if (mask == 0u || (mask >> CANRPC_MAX_SLOTS) != 0u) {
        return CANRPC_ERR_PARAM;
    }

    uint32_t start_ms = board_millis();

    for (;;) {
        canrpc_poll();

        bool busy = false;
        for (uint8_t i = 0; i < CANRPC_MAX_SLOTS; i++) {
            if ((mask & CANRPC_H(i)) == 0u) {
                continue;
            }
            if (g_slot[i].state == SLOT_WAIT_ACK || g_slot[i].state == SLOT_WAIT_DONE) {
                busy = true;
                break;
            }
        }

        if (!busy) {
            break;
        }

        if ((uint32_t)(board_millis() - start_ms) > timeout_ms) {
            for (uint8_t i = 0; i < CANRPC_MAX_SLOTS; i++) {
                if ((mask & CANRPC_H(i)) != 0u) {
                    g_slot[i].result = CANRPC_RES_TIMEOUT;
                    g_slot[i].state = SLOT_FREE;
                }
            }
            return CANRPC_ERR_TIMEOUT;
        }

        board_idle();
    }

    int rc = 0;
    for (uint8_t i = 0; i < CANRPC_MAX_SLOTS; i++) {
        if ((mask & CANRPC_H(i)) == 0u) {
            continue;
        }
        if (rc == 0 && g_slot[i].result != CANRPC_OK) {
            rc = (int)i + 1;
        }
        g_slot[i].state = SLOT_FREE;
    }
    return rc;
}

uint8_t canrpc_result(int h)
{
    if (h < 0 || h >= (int)CANRPC_MAX_SLOTS) {
        return CANRPC_RES_INVALID;
    }
    return g_slot[h].result;
}

int32_t canrpc_ret(int h)
{
    if (h < 0 || h >= (int)CANRPC_MAX_SLOTS) {
        return 0;
    }
    return g_slot[h].ret;
}

int canrpc_call_wait(uint8_t node, uint8_t cmd, int32_t arg, uint32_t timeout_ms)
{
    int h = canrpc_call(node, cmd, arg);
    if (h < 0) {
        return CANRPC_ERR_PARAM;
    }
    return canrpc_wait(CANRPC_H(h), timeout_ms);
}

int canrpc_call_wait_ret(uint8_t node, uint8_t cmd, int32_t arg, int32_t *ret, uint32_t timeout_ms)
{
    int h = canrpc_call(node, cmd, arg);
    if (h < 0) {
        return CANRPC_ERR_PARAM;
    }

    int rc = canrpc_wait(CANRPC_H(h), timeout_ms);
    if (rc == 0 && ret != NULL) {
        *ret = canrpc_ret(h);
    }
    return rc;
}

bool canrpc_publish(uint8_t topic, const void *data, uint8_t len)
{
    if (len > CANRPC_PUB_MAX_LEN) {
        return false;
    }

    uint8_t p[CANRPC_FRAME_MAX_LEN] = {0};
    p[0] = topic;
#if CANRPC_CANFD
    p[1] = len;
    if (len > 0u && data != NULL) {
        memcpy(&p[2], data, len);
    }
    return tx_frame(CANRPC_ID(CANRPC_TYPE_PUB, g_own_addr), p, (uint8_t)(2u + len));
#else
    if (len > 0u && data != NULL) {
        memcpy(&p[1], data, len);
    }
    return tx_frame(CANRPC_ID(CANRPC_TYPE_PUB, g_own_addr), p, (uint8_t)(1u + len));
#endif
}

void canrpc_set_pub_handler(canrpc_pub_cb_t cb)
{
    g_pub_cb = cb;
}

uint32_t canrpc_pub_dropped(void)
{
    return g_pub_drop;
}
