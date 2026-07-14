#include "encoder.h"
#include "config.h"
#include "fw_log.h"
#include "driver/pulse_cnt.h"

#define PCNT_HIGH_LIMIT  30000
#define PCNT_LOW_LIMIT  -30000

static pcnt_unit_handle_t s_unit = NULL;

void encoder_init(void) {
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit         = PCNT_LOW_LIMIT;
    unit_cfg.high_limit        = PCNT_HIGH_LIMIT;
    unit_cfg.flags.accum_count = 1;
    pcnt_new_unit(&unit_cfg, &s_unit);

    pcnt_chan_config_t chan_cfg = {};
    chan_cfg.edge_gpio_num  = EA;
    chan_cfg.level_gpio_num = EB;
    pcnt_channel_handle_t chan;
    pcnt_new_channel(s_unit, &chan_cfg, &chan);
    pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // watch_point enables the high/low limit hardware events so the
    // accum_count ISR fires and pcnt_unit_get_count returns the true
    // accumulated value beyond the 16-bit hardware counter range.
    pcnt_unit_add_watch_point(s_unit, PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(s_unit, PCNT_LOW_LIMIT);

    pcnt_unit_enable(s_unit);
    pcnt_unit_clear_count(s_unit);
    pcnt_unit_start(s_unit);
    FW_LOGF("INFO: encoder init OK\n");
}

int32_t encoder_get_count(void) {
    int val = 0;
    pcnt_unit_get_count(s_unit, &val);
    return (int32_t)val;
}

void encoder_clear(void) {
    pcnt_unit_clear_count(s_unit);
}
