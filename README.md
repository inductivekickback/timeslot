This wrapper is intended to simplify the [Timeslot interface](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrfxlib/mpsl/doc/timeslot.html) that is built into the [MPSL](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrfxlib/mpsl/README.html) in Nordic's [nRF Connect SDK](https://github.com/nrfconnect/sdk-nrf). If a library like [ESB](http://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.6.0/nrf/ug_esb.html) is going to be used then a preprocessor symbol called TIMESLOT_CALLS_RADIO_IRQHANDLER can be set to call RADIO_IRQHandler directly from the MPSL_TIMESLOT_SIGNAL_RADIO signal. A "skipped" callback is also provided so proprietary networks can stay in sync when timeslots are blocked or cancelled. 

---
### Usage

The timeslot_config struct and timeslot_cb structs are required.
```
#include <timeslot.h>
...
static void timeslot_err_cb(int err)
{
    LOG_ERR("Timeslot session error: %d", err);
}

static void timeslot_start_cb(void)
{
    LOG_DBG("Timeslot start");
}

static void timeslot_end_cb(void)
{
    LOG_DBG("Timeslot end");
}

static void timeslot_skipped_cb(uint8_t count)
{
    LOG_INF("Timeslot skipped: %d", count);
}

static void timeslot_stopped_cb(void)
{
    LOG_INF("Timeslot stopped");
    timeslots_start(next_interval);
}

#if !TIMESLOT_CALLS_RADIO_IRQHANDLER
static void radio_irq_cb(void)
{
    LOG_DBG("Radio_IRQHandler");
}
#endif

static struct timeslot_cb timeslot_callbacks = {
    .error     = timeslot_err_cb,
    .start     = timeslot_start_cb,
    .end       = timeslot_end_cb,
    .skipped   = timeslot_skipped_cb,
    .stopped   = timeslot_stopped_cb,
#if !TIMESLOT_CALLS_RADIO_IRQHANDLER
    .radio_irq = radio_irq_cb
#endif
};

static struct timeslot_config timeslot_config = TS_DEFAULT_CONFIG;
...
void main(void)
{
    int err = timeslot_open(&timeslot_config, &timeslot_callbacks);
    if (err) {
        LOG_ERR("timeslot_open failed (err: %d)", err);
    }
...
```
A good place to start requesting timeslots is after the BLE connection parameters have settled.
```
static void conn_param_update(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
...
    int err = timeslot_start(500);
    if (err) {
        LOG_ERR("timeslot_start failed (err=%d)", err);
    }
```
If the connection parameters change during the connection then the current timeslots can be stopped so they can be restarted in the "stopped" callback.
```
next_interval = interval;
int err = timeslot_stop();
if (err) {
    LOG_ERR("timeslot_stop failed (err=%d)", err);
}
```
