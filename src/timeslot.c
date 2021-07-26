/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <stdio.h>

#include <logging/log.h>

#define LOG_MODULE_NAME timeslot
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <mpsl.h>
#include <mpsl_timeslot.h>

#define TS_GPIO_DEBUG 1

#if TS_GPIO_DEBUG
#include <hal/nrf_gpio.h>
#define TIMESLOT_OPEN_PIN          4
#define TIMESLOT_BLOCKED_PIN       28
#define TIMESLOT_CANCELLED_PIN     30
#endif

#include <timeslot.h>

#define TIMESLOT_THREAD_STACK_SIZE 1024
#define TIMESLOT_THREAD_PRIORITY   5

enum SIGNAL_CODE
{
    SIGNAL_CODE_START             = 0x00,
    SIGNAL_CODE_TIMER0            = 0x01,
    SIGNAL_CODE_RADIO             = 0x02,
    SIGNAL_CODE_BLOCKED_CANCELLED = 0x03,
    SIGNAL_CODE_OVERSTAYED        = 0x04,
    SIGNAL_CODE_IDLE              = 0x05,
    SIGNAL_CODE_UNEXPECTED        = 0x06
};

static uint32_t                conn_interval_us;
static uint32_t                ts_len_us;
static uint8_t                 blocked_cancelled_count;
static bool                    session_open;
static bool                    timeslot_anchored;
static bool                    timeslot_started;
static bool                    timeslot_stopping;
static struct timeslot_config *p_timeslot_config;
static struct timeslot_cb     *p_timeslot_callbacks;

static struct k_poll_signal timeslot_sig = K_POLL_SIGNAL_INITIALIZER(timeslot_sig);
static struct k_poll_event  events[1]    = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &timeslot_sig, 0),
};

static mpsl_timeslot_session_id_t mpsl_session_id;

/* NOTE: MPSL return params must be in static scope. */
static mpsl_timeslot_request_t request_earliest = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
    .params.earliest = {
        .priority   = MPSL_TIMESLOT_PRIORITY_NORMAL,
    }
};

static mpsl_timeslot_request_t request_normal = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_NORMAL
};

static mpsl_timeslot_signal_return_param_t action_none = {
    .callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE
};

static mpsl_timeslot_signal_return_param_t action_end = {
    .callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END
};

static mpsl_timeslot_signal_return_param_t action_request_normal = {
    .callback_action       = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST,
    .params.request.p_next = &request_normal
};

static mpsl_timeslot_signal_return_param_t*
mpsl_cb(mpsl_timeslot_session_id_t session_id, uint32_t signal)
{
    switch (signal) {
    case MPSL_TIMESLOT_SIGNAL_START:
#if TS_GPIO_DEBUG
        nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 1);
#endif
        if (timeslot_stopping) {
#if TS_GPIO_DEBUG
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
            nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 1);
#endif
            return &action_end;
        }

        /* TIMER0 is pre-configured for 1MHz mode by the MPSL. */
        NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);
        NRF_TIMER0->CC[0]    = (ts_len_us - p_timeslot_config->safety_margin_us);
        NVIC_EnableIRQ(TIMER0_IRQn);
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_START);
        break;

    case MPSL_TIMESLOT_SIGNAL_TIMER0:
#if TS_GPIO_DEBUG
        nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
#endif
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_TIMER0);
        if (timeslot_stopping) {
            return &action_end;
        }
        request_normal.params.normal.distance_us = conn_interval_us;
        request_normal.params.normal.priority    = MPSL_TIMESLOT_PRIORITY_NORMAL;
        return &action_request_normal;

    case MPSL_TIMESLOT_SIGNAL_RADIO:
        if (timeslot_stopping) {
            return &action_end;
        }
#if TIMESLOT_CALLS_RADIO_IRQHANDLER
        RADIO_IRQHandler();
#else
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_RADIO);
#endif
        break;

    case MPSL_TIMESLOT_SIGNAL_BLOCKED:
#if TS_GPIO_DEBUG
        nrf_gpio_pin_write(TIMESLOT_BLOCKED_PIN, 1);
#endif
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_BLOCKED_CANCELLED);
        break;

    case MPSL_TIMESLOT_SIGNAL_CANCELLED:
#if TS_GPIO_DEBUG
        nrf_gpio_pin_write(TIMESLOT_CANCELLED_PIN, 1);
#endif
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_BLOCKED_CANCELLED);
        break;

    case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_IDLE);
        break;

    case MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED:
        /* Intentional fall-through */
    case MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED:
        /* Intentional fall-through */
    case MPSL_TIMESLOT_SIGNAL_INVALID_RETURN:
        /* Intentional fall-through */
    case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_UNEXPECTED);
        break;

    case MPSL_TIMESLOT_SIGNAL_OVERSTAYED:
        k_poll_signal_raise(&timeslot_sig, SIGNAL_CODE_OVERSTAYED);
        break;

    default:
        break;
    };

    return &action_none;
}

int timeslot_stop(void)
{
    if (!session_open || !timeslot_started) {
        return -TIMESLOT_ERROR_NO_TIMESLOT_STARTED;
    }
    timeslot_stopping = true;
    LOG_INF("timeslot_stop()");
    return 0;
}

int timeslot_start(uint32_t len_us, uint32_t interval_us)
{
    if (!session_open || timeslot_started || timeslot_stopping) {
        return -TIMESLOT_ERROR_TIMESLOT_ALREADY_STARTED;
    }

    LOG_INF("timeslot_start(len_us: %d, interval_us: %d)", len_us, interval_us);
    ts_len_us               = len_us;
    conn_interval_us        = interval_us;
    blocked_cancelled_count = 0;
    timeslot_started        = true;

    request_normal.params.normal.length_us     = len_us;
    request_earliest.params.earliest.length_us = len_us;

    return mpsl_timeslot_request(mpsl_session_id, &request_earliest);
}

int timeslot_open(struct timeslot_config *p_config, struct timeslot_cb *p_cb)
{
    if (session_open) {
        return -TIMESLOT_ERROR_SESSION_ALREADY_OPENED;
    }

    if (0 == p_config) {
        return -TIMESLOT_ERROR_INVALID_PARAM;
    }

    if ((0 == p_cb) || (0 == p_cb->error) || (0 == p_cb->start) || (0 == p_cb->end)) {
        return -TIMESLOT_ERROR_INVALID_PARAM;
    }
#if !TIMESLOT_CALLS_RADIO_IRQHANDLER
    if (0 == p_cb->radio_irq) {
        return -TIMESLOT_ERROR_INVALID_PARAM;
    }
#endif

    LOG_INF("timeslot_open(...)");
    p_timeslot_config    = p_config;
    p_timeslot_callbacks = p_cb;

    request_normal.params.normal.hfclk          = p_timeslot_config->hfclk;
    request_earliest.params.earliest.hfclk      = p_timeslot_config->hfclk;
    request_earliest.params.earliest.timeout_us = p_timeslot_config->timeout_us;

    int err = mpsl_timeslot_session_open(mpsl_cb, &mpsl_session_id);
    if (err) {
        return err;
    }

#if TS_GPIO_DEBUG
    nrf_gpio_cfg_output(TIMESLOT_OPEN_PIN);
    nrf_gpio_cfg_output(TIMESLOT_BLOCKED_PIN);
    nrf_gpio_cfg_output(TIMESLOT_CANCELLED_PIN);
    nrf_gpio_pin_clear(TIMESLOT_OPEN_PIN);
    nrf_gpio_pin_clear(TIMESLOT_BLOCKED_PIN);
    nrf_gpio_pin_clear(TIMESLOT_CANCELLED_PIN);
#endif

    session_open = true;
    return 0;
}

static void timeslot_stopped(void) {
#if TS_GPIO_DEBUG
    nrf_gpio_pin_write(TIMESLOT_OPEN_PIN, 0);
#endif
    timeslot_stopping = false;
    timeslot_started  = false;
    timeslot_anchored = false;
    p_timeslot_callbacks->stopped();
}

static void timeslot_thread_fn(void)
{
    int err;

    while (true) {
        k_poll(events, 1, K_FOREVER);

        switch (events[0].signal->result) {
        case SIGNAL_CODE_START:
            p_timeslot_callbacks->start();
            blocked_cancelled_count = 0;
            timeslot_anchored       = true;
            break;

        case SIGNAL_CODE_TIMER0:
            p_timeslot_callbacks->end();
            break;

#if !TIMESLOT_CALLS_RADIO_IRQHANDLER
        case SIGNAL_CODE_RADIO:
            p_timeslot_callbacks->radio_irq();
            break;
#endif

        case SIGNAL_CODE_BLOCKED_CANCELLED:
#if TS_GPIO_DEBUG
            nrf_gpio_pin_write(TIMESLOT_BLOCKED_PIN,   0);
            nrf_gpio_pin_write(TIMESLOT_CANCELLED_PIN, 0);
#endif
            blocked_cancelled_count++;
            if (blocked_cancelled_count > p_timeslot_config->skipped_tolerance) {
                if (timeslot_anchored) {
                    p_timeslot_callbacks->error(-TIMESLOT_ERROR_CANCELLED);
                } else {
                    p_timeslot_callbacks->error(-TIMESLOT_ERROR_ANCHOR_FAILED);
                }
                return;
            }
            if (timeslot_stopping) {
                timeslot_stopped();
                break;
            }
            if (timeslot_anchored) {
                request_normal.params.normal.distance_us = 
                        (conn_interval_us * (blocked_cancelled_count + 1));
                request_normal.params.normal.priority    = MPSL_TIMESLOT_PRIORITY_HIGH;
                err = mpsl_timeslot_request(mpsl_session_id, &request_normal);
            } else {
                err = mpsl_timeslot_request(mpsl_session_id, &request_earliest);
            }
            if (err) {
                timeslot_started  = false;
                timeslot_stopping = false;
                p_timeslot_callbacks->error(err);
            }
            p_timeslot_callbacks->skipped(blocked_cancelled_count);
            break;

        case SIGNAL_CODE_IDLE:
            if (timeslot_stopping) {
                timeslot_stopped();
            } else {
                /* Session ended unexpectedly */
                p_timeslot_callbacks->error(-TIMESLOT_ERROR_INTERNAL);
            }
            break;

        case SIGNAL_CODE_OVERSTAYED:
            /* This is the most probable of the what-could-go-wrong scenarios. */
            p_timeslot_callbacks->error(-TIMESLOT_ERROR_OVERSTAYED);
            break;

        case SIGNAL_CODE_UNEXPECTED:
            /* Something like MPSL_TIMESLOT_SIGNAL_INVALID_RETURN happened. */
            p_timeslot_callbacks->error(-TIMESLOT_ERROR_INTERNAL);
            break;

        default:
            p_timeslot_callbacks->error(-TIMESLOT_ERROR_INTERNAL);
            break;
        }

        events[0].signal->signaled = 0;
        events[0].state            = K_POLL_STATE_NOT_READY;
    }
}

K_THREAD_DEFINE(timeslot_thread, TIMESLOT_THREAD_STACK_SIZE,
                    timeslot_thread_fn, NULL, NULL, NULL,
                    K_PRIO_COOP(TIMESLOT_THREAD_PRIORITY), 0, 0);
