// Load Cell based end stops.
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include "trsync.h" // trsync_do_trigger
#include "board/misc.h" // timer_read_time
#include "sos_filter.h" // fixedQ12_t
#include "load_cell_probe.h" //load_cell_probe_report_sample
#include <stdint.h> // int32_t
#include <stdlib.h> // abs

// Q2.29
typedef int32_t fixedQ2_t;
#define FIXEDQ2 2
#define FIXEDQ2_FRAC_BITS ((32 - FIXEDQ2) - 1)

// Q32.29 - a Q2.29 value stored in int64
typedef int64_t fixedQ32_t;
#define FIXEDQ32_FRAC_BITS FIXEDQ2_FRAC_BITS

// Q16.15
typedef int32_t fixedQ16_t;
#define FIXEDQ16 16
#define FIXEDQ16_FRAC_BITS ((32 - FIXEDQ16) - 1)

// Q48.15 - a Q16.15 value stored in int64
typedef int64_t fixedQ48_t;
#define FIXEDQ48_FRAC_BITS FIXEDQ16_FRAC_BITS

#define MAX_TRIGGER_GRAMS ((1UL << FIXEDQ16) - 1)
#define ERROR_SAFETY_RANGE 0
#define ERROR_OVERFLOW 1
#define ERROR_WATCHDOG 2

// Creality PRTouch-compatible waveform trigger.
// V3 SE HX711 runs at approximately 80Hz (12.5ms/sample).
#define PRTOUCH_BUF_LEN 32
#define PRTOUCH_Q16_ONE 65536LL
#define PRTOUCH_HP_COEFF_Q16 56639LL
#define PRTOUCH_LP_K1_Q16 45875LL
#define TRIGGER_SOURCE_PRTOUCH 1
#define TRIGGER_SOURCE_FORCE 2

// Flags
enum {FLAG_IS_HOMING = 1 << 0
    , FLAG_IS_HOMING_TRIGGER = 1 << 1
    , FLAG_AWAIT_HOMING = 1 << 2
    };

// Endstop Structure
struct load_cell_probe {
    struct timer time;
    uint32_t trigger_grams, trigger_ticks, last_sample_ticks, rest_ticks;
    uint32_t homing_start_time;
    struct trsync *ts;
    int32_t safety_counts_min, safety_counts_max, tare_counts;
    uint8_t flags, trigger_reason, error_reason, watchdog_max
            , watchdog_count;
    fixedQ16_t trigger_grams_fixed;
    fixedQ2_t grams_per_count;
    struct sos_filter *sf;

    // Optional Creality-style waveform trigger.
    uint8_t prtouch_enabled;
    uint8_t prtouch_count;
    uint8_t prtouch_index;
    uint8_t trigger_source;
    uint32_t prtouch_min_hold;
    uint32_t prtouch_max_hold;
    int32_t prtouch_buf[PRTOUCH_BUF_LEN];
    uint32_t prtouch_ticks[PRTOUCH_BUF_LEN];
};

static inline uint8_t
overflows_int32(int64_t value) {
    return value > (int64_t)INT32_MAX || value < (int64_t)INT32_MIN;
}

// returns the integer part of a fixedQ48_t
static inline int64_t
round_fixedQ48(const int64_t fixed_value) {
    return fixed_value >> FIXEDQ48_FRAC_BITS;
}

// Convert sensor counts to grams
static inline fixedQ48_t
counts_to_grams(struct load_cell_probe *lcp, const int32_t counts) {
    // tearing ensures readings are referenced to 0.0g
    const int32_t delta = counts - lcp->tare_counts;
    // convert sensor counts to grams by multiplication: 124 * 0.051 = 6.324
    // this optimizes to single cycle SMULL instruction
    const fixedQ32_t product = (int64_t)delta * (int64_t)lcp->grams_per_count;
    // after multiplication there are 30 fraction bits, reduce to 15
    // caller verifies this wont overflow a 32bit int when truncated
    const fixedQ48_t grams = product >>
                                (FIXEDQ32_FRAC_BITS - FIXEDQ48_FRAC_BITS);
    return grams;
}

static inline uint8_t
is_flag_set(const uint8_t mask, struct load_cell_probe *lcp)
{
    return !!(mask & lcp->flags);
}

static inline void
set_flag(uint8_t mask, struct load_cell_probe *lcp)
{
    lcp->flags |= mask;
}

static inline void
clear_flag(uint8_t mask, struct load_cell_probe *lcp)
{
    lcp->flags &= ~mask;
}

void
try_trigger(struct load_cell_probe *lcp, uint32_t ticks, uint8_t source)
{
    uint8_t is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp);
    if (!is_homing_triggered) {
        // the first triggering sample when homing sets the trigger time
        lcp->trigger_ticks = ticks;
        lcp->trigger_source = source;
        // this flag latches until a reset, disabling further triggering
        set_flag(FLAG_IS_HOMING_TRIGGER, lcp);
        trsync_do_trigger(lcp->ts, lcp->trigger_reason);
    }
}

void
trigger_error(struct load_cell_probe *lcp, uint8_t error_code)
{
    trsync_do_trigger(lcp->ts, lcp->error_reason + error_code);
}

// Return absolute magnitude without overflowing INT32_MIN.
static inline int64_t
prtouch_abs32(int32_t value)
{
    return value < 0 ? -(int64_t)value : (int64_t)value;
}

static void
prtouch_reset(struct load_cell_probe *lcp)
{
    lcp->prtouch_count = 0;
    lcp->prtouch_index = 0;
    for (uint8_t i = 0; i < PRTOUCH_BUF_LEN; i++) {
        lcp->prtouch_buf[i] = 0;
        lcp->prtouch_ticks[i] = 0;
    }
}

static void
prtouch_add_sample(
    struct load_cell_probe *lcp, int32_t value, uint32_t ticks)
{
    lcp->prtouch_buf[lcp->prtouch_index] = value;
    lcp->prtouch_ticks[lcp->prtouch_index] = ticks;

    lcp->prtouch_index =
        (lcp->prtouch_index + 1) % PRTOUCH_BUF_LEN;

    if (lcp->prtouch_count < PRTOUCH_BUF_LEN)
        lcp->prtouch_count++;
}

// Creality non-ADC PRTouch trigger semantics.
//
// Signal path:
// raw HX711 -> tare subtraction -> 32 sample FIFO
// -> polarity normalization -> 3-point min-absolute filter
// -> high-pass -> recursive low-pass
// -> max-hold OR rising/dominance/min-hold trigger.
//
// Filters are implemented in Q16 fixed point to avoid floating-point
// work inside the MCU sampling path.
static uint8_t
prtouch_check_trigger(struct load_cell_probe *lcp)
{
    if (lcp->prtouch_count < PRTOUCH_BUF_LEN)
        return 0;

    // prtouch_index points to the oldest sample once the ring is full.
    uint8_t base = lcp->prtouch_index;

    // Creality polarity normalization is based on the original
    // first and last samples before filtering.
    int32_t first = lcp->prtouch_buf[base];
    uint8_t last_idx =
        (base + PRTOUCH_BUF_LEN - 1) % PRTOUCH_BUF_LEN;
    int32_t last = lcp->prtouch_buf[last_idx];
    uint8_t invert = last < first;

    // Keep a three-sample sliding window instead of copying the
    // entire 32-sample FIFO onto the MCU stack.
    int32_t s0 = first;
    int32_t s1 =
        lcp->prtouch_buf[(base + 1) % PRTOUCH_BUF_LEN];
    int32_t s2 =
        lcp->prtouch_buf[(base + 2) % PRTOUCH_BUF_LEN];

    if (invert) {
        s0 = -s0;
        s1 = -s1;
        s2 = -s2;
    }

    // Stream the Creality high-pass and recursive low-pass filters.
    //
    // hp_prev is the previous HIGH-PASS output.
    // lp_prev is the previous LOW-PASS output.
    //
    // This is mathematically equivalent to the former two-array
    // implementation, but does not retain all 32 filtered values.
    int32_t prev_data = 0;
    int64_t hp_prev = 0;
    int64_t lp_prev = 0;

    // filtered[0] in the original implementation was zero, so include
    // zero in the maximum of samples 0..28.
    int64_t earlier_max = 0;

    int64_t p29 = 0;
    int64_t p30 = 0;
    int64_t p31 = 0;

    for (uint8_t i = 0; i < PRTOUCH_BUF_LEN; i++) {
        int32_t data_i;

        // Creality non-ADC T filter. The final two samples remain
        // unfiltered, matching the original in-place implementation.
        if (i < PRTOUCH_BUF_LEN - 2) {
            int32_t chosen =
                prtouch_abs32(s0) < prtouch_abs32(s1)
                ? s0 : s1;

            if (prtouch_abs32(s2) < prtouch_abs32(chosen))
                chosen = s2;

            data_i = chosen;
        } else {
            data_i = s0;
        }

        // filtered[0] is zero in the original implementation.
        if (i != 0) {
            int64_t input =
                ((int64_t)data_i - (int64_t)prev_data)
                * PRTOUCH_Q16_ONE;
            input += hp_prev;

            int64_t hp =
                (input * PRTOUCH_HP_COEFF_Q16) >> 16;

            int64_t lp =
                (
                    hp * PRTOUCH_LP_K1_Q16
                    + lp_prev
                      * (PRTOUCH_Q16_ONE - PRTOUCH_LP_K1_Q16)
                ) >> 16;

            hp_prev = hp;
            lp_prev = lp;

            if (i <= PRTOUCH_BUF_LEN - 4) {
                if (lp > earlier_max)
                    earlier_max = lp;
            } else if (i == PRTOUCH_BUF_LEN - 3) {
                p29 = lp;
            } else if (i == PRTOUCH_BUF_LEN - 2) {
                p30 = lp;
            } else {
                p31 = lp;
            }
        }

        prev_data = data_i;

        // Advance the three-sample raw window.
        if (i < PRTOUCH_BUF_LEN - 1) {
            s0 = s1;
            s1 = s2;

            uint8_t next_pos = i + 3;
            if (next_pos < PRTOUCH_BUF_LEN) {
                uint8_t idx =
                    (base + next_pos) % PRTOUCH_BUF_LEN;
                s2 = lcp->prtouch_buf[idx];

                if (invert)
                    s2 = -s2;
            }
        }
    }

    int64_t min_hold =
        (int64_t)lcp->prtouch_min_hold * PRTOUCH_Q16_ONE;
    int64_t max_hold =
        (int64_t)lcp->prtouch_max_hold * PRTOUCH_Q16_ONE;

    // Factory path 1: max-hold fast trigger.
    if (p31 > max_hold
        && p30 > max_hold / 2
        && p29 > max_hold / 3)
        return 1;

    // Factory path 2: final three samples must be rising.
    if (!(p31 > p30 && p30 > p29))
        return 0;

    // Factory dominance rule: all final three samples must exceed
    // every filtered sample from indices 0 through 28.
    if (!(p31 > earlier_max
          && p30 > earlier_max
          && p29 > earlier_max))
        return 0;

    // Factory minimum hold.
    if (p31 < min_hold)
        return 0;

    return 1;
}

// Used by Sensors to report new raw ADC sample
void
load_cell_probe_report_sample(struct load_cell_probe *lcp
                                , const int32_t sample)
{
    // only process samples when homing
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lcp);
    if (!is_homing) {
        return;
    }

    // save new sample
    uint32_t ticks = timer_read_time();
    lcp->last_sample_ticks = ticks;
    lcp->watchdog_count = 0;

    // do not trigger before homing start time
    uint8_t await_homing = is_flag_set(FLAG_AWAIT_HOMING, lcp);
    if (await_homing && timer_is_before(ticks, lcp->homing_start_time)) {
        return;
    }
    clear_flag(FLAG_AWAIT_HOMING, lcp);

    // check for safety limit violations
    const uint8_t is_safety_trigger = sample <= lcp->safety_counts_min
                                        || sample >= lcp->safety_counts_max;
    // too much force, this is an error while homing
    if (is_safety_trigger) {
        trigger_error(lcp, ERROR_SAFETY_RANGE);
        return;
    }

    // Feed the raw, tare-referenced HX711 value into the optional
    // Creality waveform trigger before Kalico's grams/SOS path.
    if (lcp->prtouch_enabled
            && !is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp)) {
        int32_t prtouch_value = sample - lcp->tare_counts;
        prtouch_add_sample(lcp, prtouch_value, ticks);

        if (prtouch_check_trigger(lcp)) {
            try_trigger(
                lcp, lcp->last_sample_ticks,
                TRIGGER_SOURCE_PRTOUCH);
            return;
        }
    }

    // convert sample to grams
    const fixedQ48_t raw_grams = counts_to_grams(lcp, sample);
    if (overflows_int32(raw_grams)) {
        trigger_error(lcp, ERROR_OVERFLOW);
        return;
    }

    // perform filtering
    const fixedQ16_t filtered_grams = sosfilt(lcp->sf, (fixedQ16_t)raw_grams);

    // Use the Kalico force trigger only when Creality-compatible
    // PRTouch waveform triggering is disabled.  The independent safety
    // range above remains active in both modes.
    if (!lcp->prtouch_enabled
            && abs(filtered_grams) >= lcp->trigger_grams_fixed) {
        try_trigger(
            lcp, lcp->last_sample_ticks,
            TRIGGER_SOURCE_FORCE);
    }
}

// Timer callback that monitors for timeouts
static uint_fast8_t
watchdog_event(struct timer *t)
{
    struct load_cell_probe *lcp = container_of(t, struct load_cell_probe
                                        , time);
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lcp);
    uint8_t is_homing_trigger = is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp);
    // the watchdog stops when not homing or when trsync becomes triggered
    if (!is_homing || is_homing_trigger) {
        return SF_DONE;
    }

    if (lcp->watchdog_count > lcp->watchdog_max) {
        trigger_error(lcp, ERROR_WATCHDOG);
    }
    lcp->watchdog_count += 1;

    // A sample was recently delivered, continue monitoring
    lcp->time.waketime += lcp->rest_ticks;
    return SF_RESCHEDULE;
}

static void
set_endstop_range(struct load_cell_probe *lcp
                , int32_t safety_counts_min, int32_t safety_counts_max
                , int32_t tare_counts, uint32_t trigger_grams
                , fixedQ2_t grams_per_count)
{
    if (!(safety_counts_max >= safety_counts_min)) {
        shutdown("Safety range reversed");
    }
    if (trigger_grams > MAX_TRIGGER_GRAMS) {
        shutdown("trigger_grams too large");
    }
    // grams_per_count must be a positive fraction in Q2 format
    const fixedQ2_t one = 1UL << FIXEDQ2_FRAC_BITS;
    if (grams_per_count < 0 || grams_per_count >= one) {
        shutdown("grams_per_count is invalid");
    }
    lcp->safety_counts_min = safety_counts_min;
    lcp->safety_counts_max = safety_counts_max;
    lcp->tare_counts = tare_counts;
    lcp->trigger_grams = trigger_grams;
    lcp->trigger_grams_fixed = trigger_grams << FIXEDQ16_FRAC_BITS;
    lcp->grams_per_count = grams_per_count;
}

// Create a load_cell_probe
void
command_config_load_cell_probe(uint32_t *args)
{
    struct load_cell_probe *lcp = oid_alloc(args[0]
                            , command_config_load_cell_probe, sizeof(*lcp));
    lcp->flags = 0;
    lcp->trigger_ticks = 0;
    lcp->watchdog_max = 0;
    lcp->watchdog_count = 0;
    lcp->prtouch_enabled = 0;
    lcp->prtouch_min_hold = 1000;
    lcp->prtouch_max_hold = 1500;
    lcp->trigger_source = 0;
    prtouch_reset(lcp);
    lcp->sf = sos_filter_oid_lookup(args[1]);
    set_endstop_range(lcp, 0, 0, 0, 0, 0);
}
DECL_COMMAND(command_config_load_cell_probe, "config_load_cell_probe"
                                               " oid=%c sos_filter_oid=%c");

// Lookup a load_cell_probe
struct load_cell_probe *
load_cell_probe_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_probe);
}

// Set the triggering range and tare value
void
command_load_cell_probe_set_range(uint32_t *args)
{
    struct load_cell_probe *lcp = load_cell_probe_oid_lookup(args[0]);
    set_endstop_range(lcp, args[1], args[2], args[3], args[4]
                , (fixedQ16_t)args[5]);
}
DECL_COMMAND(command_load_cell_probe_set_range, "load_cell_probe_set_range"
    " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i"
    " trigger_grams=%u grams_per_count=%i");

// Enable or disable Creality-compatible waveform triggering.
// min_hold/max_hold remain in Creality filtered sensor units.
void
command_load_cell_probe_set_prtouch(uint32_t *args)
{
    struct load_cell_probe *lcp =
        load_cell_probe_oid_lookup(args[0]);

    lcp->prtouch_enabled = args[1];
    lcp->prtouch_min_hold = args[2];
    lcp->prtouch_max_hold = args[3];
    prtouch_reset(lcp);
}
DECL_COMMAND(
    command_load_cell_probe_set_prtouch,
    "load_cell_probe_set_prtouch"
    " oid=%c enabled=%c min_hold=%u max_hold=%u");

// Home an axis
void
command_load_cell_probe_home(uint32_t *args)
{
    struct load_cell_probe *lcp = load_cell_probe_oid_lookup(args[0]);
    sched_del_timer(&lcp->time);
    // clear the homing trigger flag
    clear_flag(FLAG_IS_HOMING_TRIGGER, lcp);
    clear_flag(FLAG_IS_HOMING, lcp);
    lcp->trigger_ticks = 0;
    lcp->trigger_source = 0;
    prtouch_reset(lcp);
    lcp->ts = NULL;
    // 0 samples indicates homing is finished
    if (args[3] == 0) {
        // Disable end stop checking
        return;
    }
    lcp->ts = trsync_oid_lookup(args[1]);
    lcp->trigger_reason = args[2];
    lcp->error_reason = args[3];
    lcp->time.waketime = args[4];
    lcp->homing_start_time = args[4];
    lcp->rest_ticks = args[5];
    lcp->watchdog_max = args[6];
    lcp->watchdog_count = 0;
    lcp->time.func = watchdog_event;
    set_flag(FLAG_IS_HOMING, lcp);
    set_flag(FLAG_AWAIT_HOMING, lcp);
    sched_add_timer(&lcp->time);
}
DECL_COMMAND(command_load_cell_probe_home,
             "load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c"
             " error_reason=%c clock=%u rest_ticks=%u timeout=%u");

void
command_load_cell_probe_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell_probe *lcp = load_cell_probe_oid_lookup(args[0]);
    sendf("load_cell_probe_state oid=%c is_homing_trigger=%c trigger_ticks=%u"
          " trigger_source=%c"
            , oid
            , is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp)
            , lcp->trigger_ticks
            , lcp->trigger_source);
}
DECL_COMMAND(command_load_cell_probe_query_state
                , "load_cell_probe_query_state oid=%c");

// Query one sample from the frozen Creality-compatible PRTouch FIFO.
//
// "index" is chronological:
//   0  = oldest sample in the trigger window
//   31 = newest sample that caused the PRTouch trigger
//
// The ring stops advancing as soon as FLAG_IS_HOMING_TRIGGER is set,
// so these values and MCU timestamps represent the exact trigger FIFO.
void
command_load_cell_probe_query_prtouch_sample(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t logical_index = args[1];

    struct load_cell_probe *lcp =
        load_cell_probe_oid_lookup(oid);

    uint8_t valid =
        logical_index < lcp->prtouch_count
        && logical_index < PRTOUCH_BUF_LEN;

    int32_t sample = 0;
    uint32_t ticks = 0;

    if (valid) {
        uint8_t base =
            lcp->prtouch_count < PRTOUCH_BUF_LEN
            ? 0
            : lcp->prtouch_index;

        uint8_t physical_index =
            (base + logical_index) % PRTOUCH_BUF_LEN;

        sample = lcp->prtouch_buf[physical_index];
        ticks = lcp->prtouch_ticks[physical_index];
    }

    sendf(
        "load_cell_probe_prtouch_sample"
        " oid=%c index=%c valid=%c sample=%i ticks=%u",
        oid,
        logical_index,
        valid,
        sample,
        ticks);
}

DECL_COMMAND(
    command_load_cell_probe_query_prtouch_sample,
    "load_cell_probe_query_prtouch_sample oid=%c index=%c");
