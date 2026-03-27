/*
 * GPIO Timing-Protocol Executor (GTPE) — SimAVR
 *
 * Data-driven virtual GPIO device engine. Reads JSON timing scripts
 * at startup and uses SimAVR cycle timers for cycle-accurate
 * pin responses.
 *
 * Copyright (c) 2025-2026 Dustalon Project
 */

#include "gpio_timing_executor.h"
#include "sim_io.h"
#include "cJSON.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── Forward declarations ────────────────────────────────────────────────── */

static avr_cycle_count_t gte_step_callback(avr_t *avr,
                                            avr_cycle_count_t when,
                                            void *param);
static void gte_trigger_notify(struct avr_irq_t *irq, uint32_t value,
                                void *param);
static void gte_execute_step(GteDevice *dev);

/* ── Simple random noise (stdlib-based) ──────────────────────────────────── */

static float gte_rand_range(float lo, float hi)
{
    float t = (float)rand() / (float)RAND_MAX;
    return lo + t * (hi - lo);
}

/* ── Data Encoders ───────────────────────────────────────────────────────── */

static void gte_encode_dht11(GteDevice *dev)
{
    float temp = dev->params[0]; /* temperature */
    float hum  = dev->params[1]; /* humidity */

    /* Apply noise if params exist */
    if (dev->num_params > 2 && dev->params[2] > 0) {
        temp += gte_rand_range(-dev->params[2], dev->params[2]);
    }
    if (dev->num_params > 3 && dev->params[3] > 0) {
        hum += gte_rand_range(-dev->params[3], dev->params[3]);
    }

    /* Clamp to DHT11 range */
    if (temp < 0) temp = 0; if (temp > 50) temp = 50;
    if (hum < 20) hum = 20; if (hum > 90) hum = 90;

    uint8_t hum_int  = (uint8_t)hum;
    uint8_t hum_dec  = 0;
    uint8_t temp_int = (uint8_t)temp;
    uint8_t temp_dec = 0;

    dev->data_bytes[0] = hum_int;
    dev->data_bytes[1] = hum_dec;
    dev->data_bytes[2] = temp_int;
    dev->data_bytes[3] = temp_dec;
    dev->data_bytes[4] = (hum_int + hum_dec + temp_int + temp_dec) & 0xFF;
    dev->data_bit_count = 40;
}

void gte_encode_data(GteDevice *dev)
{
    switch (dev->encoder_type) {
    case GTE_ENCODER_DHT11_40BIT:
        gte_encode_dht11(dev);
        break;
    case GTE_ENCODER_RAW_PULSE:
    case GTE_ENCODER_NONE:
    default:
        break;
    }
}

/* ── Delay Computation ───────────────────────────────────────────────────── */

static uint32_t gte_compute_delay_us(GteDevice *dev, const GteStep *step)
{
    switch (step->type) {
    case GTE_STEP_SET_PIN:
    case GTE_STEP_COMMENT:
        return step->delay_us;

    case GTE_STEP_SET_PIN_EXPR: {
        float val = 0;
        int idx = step->delay_expr.param_index;
        if (idx >= 0 && idx < dev->num_params) {
            val = dev->params[idx];
        }
        /* Apply noise for HC-SR04 distance */
        if (dev->num_params > idx + 1 && dev->params[idx + 1] > 0) {
            val += gte_rand_range(-dev->params[idx + 1],
                                   dev->params[idx + 1]);
            if (val < 2.0f) val = 2.0f; /* HC-SR04 min range */
        }
        float delay_us = val * step->delay_expr.multiplier;
        if (delay_us < 0) delay_us = 0;
        return (uint32_t)delay_us;
    }

    case GTE_STEP_SET_PIN_BIT: {
        /* Look up current data bit */
        if (dev->current_bit < dev->data_bit_count) {
            int byte_idx = dev->current_bit / 8;
            int bit_idx  = 7 - (dev->current_bit % 8); /* MSB first */
            bool bit_val = (dev->data_bytes[byte_idx] >> bit_idx) & 1;
            return bit_val ? step->delay_bit_us[1] : step->delay_bit_us[0];
        }
        return step->delay_bit_us[0]; /* fallback */
    }

    default:
        return 0;
    }
}

/* ── Step Execution ──────────────────────────────────────────────────────── */

static void gte_execute_step(GteDevice *dev)
{
    if (!dev->active || !dev->avr) return;
    if (dev->current_step >= dev->num_steps) {
        /* Sequence complete — set idle state */
        for (int i = 0; i < dev->num_drive_irqs; i++) {
            avr_raise_irq(dev->drive_irqs[i], dev->idle_values[i] != 0);
        }
        dev->active = false;
        return;
    }

    GteStep *step = &dev->steps[dev->current_step];

    switch (step->type) {
    case GTE_STEP_COMMENT:
        /* Skip to next step immediately */
        dev->current_step++;
        gte_execute_step(dev);
        return;

    case GTE_STEP_SET_PIN:
    case GTE_STEP_SET_PIN_EXPR:
    case GTE_STEP_SET_PIN_BIT: {
        uint32_t delay_us = gte_compute_delay_us(dev, step);

        /* For bit steps, advance the bit counter */
        if (step->type == GTE_STEP_SET_PIN_BIT) {
            dev->current_bit++;
        }

        dev->current_step++;

        /* Schedule pin change after delay */
        if (delay_us > 0) {
            avr_cycle_timer_register_usec(dev->avr, delay_us,
                                           gte_step_callback, dev);
        } else {
            /* Zero delay — execute immediately */
            gte_step_callback(dev->avr, 0, dev);
        }
        return;
    }

    case GTE_STEP_REPEAT_START:
        dev->repeat_counter = step->repeat_count;
        dev->repeat_start_step = dev->current_step + 1;
        dev->current_step++;
        gte_execute_step(dev);
        return;

    case GTE_STEP_REPEAT_END:
        dev->repeat_counter--;
        if (dev->repeat_counter > 0) {
            dev->current_step = dev->repeat_start_step;
        } else {
            dev->current_step++;
        }
        gte_execute_step(dev);
        return;

    default:
        dev->current_step++;
        gte_execute_step(dev);
        return;
    }
}

static avr_cycle_count_t gte_step_callback(avr_t *avr,
                                            avr_cycle_count_t when,
                                            void *param)
{
    GteDevice *dev = (GteDevice *)param;
    if (!dev->active || !dev->avr) return 0;

    /* The previous step scheduled us — execute its pin change.
     * The step index was already advanced, so look at step - 1. */
    int prev = dev->current_step - 1;
    if (prev >= 0 && prev < dev->num_steps) {
        GteStep *step = &dev->steps[prev];
        if (step->pin_index >= 0 && step->pin_index < dev->num_drive_irqs) {
            avr_raise_irq(dev->drive_irqs[step->pin_index],
                           step->pin_value != 0);
        }
    }

    /* Continue to next step */
    gte_execute_step(dev);
    return 0;
}

/* ── Trigger Detection ───────────────────────────────────────────────────── */

static void gte_trigger_notify(struct avr_irq_t *irq, uint32_t value,
                                void *param)
{
    GteDevice *dev = (GteDevice *)param;
    bool high = (value != 0);

    if (dev->bidirectional) {
        /* DHT11-style: firmware pulls LOW then releases HIGH */
        if (!high && !dev->pin_was_low && !dev->active) {
            dev->pin_was_low = true;
        } else if (high && dev->pin_was_low && !dev->active) {
            dev->pin_was_low = false;
            /* Start response */
            gte_encode_data(dev);
            dev->current_bit = 0;
            dev->current_step = 0;
            dev->active = true;
            gte_execute_step(dev);
        }
    } else {
        /* HC-SR04-style: rising edge on trigger pin */
        if (high && !dev->pin_was_high && !dev->active) {
            gte_encode_data(dev);
            dev->current_bit = 0;
            dev->current_step = 0;
            dev->active = true;
            gte_execute_step(dev);
        }
        dev->pin_was_high = high;
    }
}

/* ── JSON Parsing ────────────────────────────────────────────────────────── */

static GteEncoderType gte_parse_encoder(const char *name)
{
    if (!name) return GTE_ENCODER_NONE;
    if (strcmp(name, "dht11_40bit") == 0) return GTE_ENCODER_DHT11_40BIT;
    if (strcmp(name, "raw_pulse") == 0)   return GTE_ENCODER_RAW_PULSE;
    return GTE_ENCODER_NONE;
}

/**
 * Parse "pN * constant" expression string.
 * Returns true on success.
 */
static bool gte_parse_expr(const char *expr, GteDelayExpr *out)
{
    /* Format: "p<digit> * <number>" */
    if (!expr || expr[0] != 'p') return false;

    char *endptr;
    int idx = (int)strtol(expr + 1, &endptr, 10);
    if (idx < 0 || idx >= GTE_MAX_PARAMS) return false;

    /* Skip whitespace and '*' */
    while (*endptr == ' ' || *endptr == '*') endptr++;

    float mult = strtof(endptr, NULL);
    out->param_index = idx;
    out->multiplier = mult;
    return true;
}

/**
 * Resolve a pin name (e.g., "echo", "data") to a drive_irqs index.
 */
static int gte_resolve_pin_name(const char *name,
                                 const char **pin_names, int num_pins)
{
    for (int i = 0; i < num_pins; i++) {
        if (pin_names[i] && strcmp(pin_names[i], name) == 0) return i;
    }
    return -1;
}

static int gte_parse_steps(cJSON *steps_arr, GteDevice *dev,
                            const char **pin_names, int num_pin_names,
                            cJSON *dev_json, int start_idx)
{
    int idx = start_idx;

    cJSON *step_obj = NULL;
    cJSON_ArrayForEach(step_obj, steps_arr) {
        if (idx >= GTE_MAX_STEPS) break;
        if (!cJSON_IsObject(step_obj)) continue;

        /* Check for repeat block first (repeat steps may also carry a "comment" field) */
        cJSON *repeat_item = cJSON_GetObjectItem(step_obj, "repeat");
        if (repeat_item && cJSON_IsNumber(repeat_item)) {
            int count = repeat_item->valueint;
            cJSON *seq_name_item = cJSON_GetObjectItem(step_obj, "sequence");
            const char *seq_name = seq_name_item ? seq_name_item->valuestring
                                                 : NULL;

            dev->steps[idx].type = GTE_STEP_REPEAT_START;
            dev->steps[idx].repeat_count = count;
            idx++;

            /* Inline the referenced sequence from the device JSON */
            if (seq_name && dev_json) {
                cJSON *seq_list = cJSON_GetObjectItem(dev_json, seq_name);
                if (seq_list && cJSON_IsArray(seq_list)) {
                    idx = gte_parse_steps(seq_list, dev, pin_names,
                                           num_pin_names, NULL, idx);
                }
            }

            dev->steps[idx].type = GTE_STEP_REPEAT_END;
            dev->steps[idx].repeat_jump_target = 0;
            idx++;
            continue;
        }

        /* Check for comment-only step (skip) */
        cJSON *comment_item = cJSON_GetObjectItem(step_obj, "comment");
        cJSON *set_item     = cJSON_GetObjectItem(step_obj, "set");
        if (comment_item && !set_item) {
            dev->steps[idx].type = GTE_STEP_COMMENT;
            dev->steps[idx].delay_us = 0;
            idx++;
            continue;
        }

        /* Normal step: delay + set pin */
        const char *pin_name = set_item ? set_item->valuestring : NULL;
        int pin_idx = pin_name ? gte_resolve_pin_name(pin_name, pin_names,
                                                       num_pin_names) : 0;

        int pin_val = 0;
        cJSON *value_item = cJSON_GetObjectItem(step_obj, "value");
        if (value_item && cJSON_IsNumber(value_item)) {
            pin_val = value_item->valueint;
        }

        GteStep *s = &dev->steps[idx];
        s->pin_index = pin_idx;
        s->pin_value = pin_val;

        cJSON *expr_item    = cJSON_GetObjectItem(step_obj, "delay_us_expr");
        cJSON *bit_item     = cJSON_GetObjectItem(step_obj, "delay_us_bit");
        cJSON *delay_item   = cJSON_GetObjectItem(step_obj, "delay_us");

        if (expr_item && cJSON_IsString(expr_item)) {
            s->type = GTE_STEP_SET_PIN_EXPR;
            if (!gte_parse_expr(expr_item->valuestring, &s->delay_expr)) {
                fprintf(stderr, "GTPE: failed to parse expr: %s\n",
                        expr_item->valuestring);
                s->type = GTE_STEP_SET_PIN;
                s->delay_us = 0;
            }
        } else if (bit_item && cJSON_IsArray(bit_item)) {
            s->type = GTE_STEP_SET_PIN_BIT;
            int bi = 0;
            cJSON *bval = NULL;
            cJSON_ArrayForEach(bval, bit_item) {
                if (cJSON_IsNumber(bval) && bi < 2) {
                    s->delay_bit_us[bi] = (uint32_t)bval->valueint;
                    bi++;
                }
            }
        } else if (delay_item && cJSON_IsNumber(delay_item)) {
            s->type = GTE_STEP_SET_PIN;
            s->delay_us = (uint32_t)delay_item->valueint;
        } else {
            s->type = GTE_STEP_SET_PIN;
            s->delay_us = 0;
        }

        idx++;
    }

    return idx;
}

static bool gte_parse_one_device(cJSON *dev_json, GteDevice *dev)
{
    memset(dev, 0, sizeof(*dev));

    /* Name */
    cJSON *name_item = cJSON_GetObjectItem(dev_json, "name");
    if (name_item && cJSON_IsString(name_item)) {
        strncpy(dev->name, name_item->valuestring, GTE_NAME_LEN - 1);
        dev->name[GTE_NAME_LEN - 1] = '\0';
    }

    /* Encoder */
    cJSON *enc_item = cJSON_GetObjectItem(dev_json, "dataEncoder");
    dev->encoder_type = gte_parse_encoder(
        (enc_item && cJSON_IsString(enc_item)) ? enc_item->valuestring : NULL);

    /* Pins */
    cJSON *pins_obj = cJSON_GetObjectItem(dev_json, "pins");
    if (!pins_obj || !cJSON_IsObject(pins_obj)) return false;

    /* Collect pin names — we need TWO arrays:
     *   pin_names[]       — ALL pin names (watch + drive), for reference
     *   drive_pin_names[] — only DRIVE pin names, indexed to match drive_irqs[]
     * Steps reference pins by name via "set": "echo", and the resolved index
     * must map to drive_irqs[], not pin_names[]. */
    const char *pin_names[GTE_MAX_DRIVE_PINS * 2];
    int num_pin_names = 0;
    const char *drive_pin_names[GTE_MAX_DRIVE_PINS];
    int num_drive_pin_names = 0;
    /* Track watch pin name index for bidirectional pins */
    int watch_pin_name_idx = -1;

    cJSON *pin_conf = NULL;
    cJSON_ArrayForEach(pin_conf, pins_obj) {
        if (num_pin_names >= GTE_MAX_DRIVE_PINS * 2) break;

        const char *pname = pin_conf->string;
        pin_names[num_pin_names] = pname;

        cJSON *dir_item = cJSON_GetObjectItem(pin_conf, "direction");
        const char *dir = (dir_item && cJSON_IsString(dir_item))
                              ? dir_item->valuestring : NULL;

        if (dir && strcmp(dir, "watch") == 0) {
            watch_pin_name_idx = num_pin_names;
            cJSON *edge_item = cJSON_GetObjectItem(pin_conf, "edge");
            const char *edge = (edge_item && cJSON_IsString(edge_item))
                                   ? edge_item->valuestring : NULL;
            dev->watch_rising = !edge || strcmp(edge, "rising") == 0;
        } else if (dir && strcmp(dir, "drive") == 0) {
            int di = dev->num_drive_irqs;
            drive_pin_names[num_drive_pin_names++] = pname;
            cJSON *idle_item = cJSON_GetObjectItem(pin_conf, "idle");
            if (idle_item && cJSON_IsNumber(idle_item)) {
                dev->idle_values[di] = idle_item->valueint;
            }
            dev->num_drive_irqs++;
        } else if (dir && strcmp(dir, "bidirectional") == 0) {
            /* Same pin for watch and drive */
            watch_pin_name_idx = num_pin_names;
            dev->bidirectional = true;
            cJSON *wedge_item = cJSON_GetObjectItem(pin_conf, "watch_edge");
            const char *wedge = (wedge_item && cJSON_IsString(wedge_item))
                                    ? wedge_item->valuestring : NULL;
            dev->watch_rising = wedge && strcmp(wedge, "rising") == 0;

            int di = dev->num_drive_irqs;
            drive_pin_names[num_drive_pin_names++] = pname;
            cJSON *idle_item = cJSON_GetObjectItem(pin_conf, "idle");
            if (idle_item && cJSON_IsNumber(idle_item)) {
                dev->idle_values[di] = idle_item->valueint;
            }
            dev->num_drive_irqs++;
        }

        num_pin_names++;
    }

    /* Resolve pin numbers from JSON resolvedPins */
    dev->resolved_watch_pin = -1;
    dev->num_resolved_drive_pins = 0;
    cJSON *resolved = cJSON_GetObjectItem(dev_json, "resolvedPins");
    if (resolved && cJSON_IsObject(resolved)) {
        /* Map each pin name to its resolved GPIO/Arduino number */
        for (int i = 0; i < num_pin_names; i++) {
            cJSON *rpin = cJSON_GetObjectItem(resolved, pin_names[i]);
            int pnum = (rpin && cJSON_IsNumber(rpin)) ? rpin->valueint : -1;

            if (i == watch_pin_name_idx) {
                dev->resolved_watch_pin = pnum;
            }
        }
        /* Drive pins: resolve in drive_pin_names order */
        for (int i = 0; i < num_drive_pin_names; i++) {
            cJSON *rpin = cJSON_GetObjectItem(resolved, drive_pin_names[i]);
            dev->resolved_drive_pins[i] =
                (rpin && cJSON_IsNumber(rpin)) ? rpin->valueint : -1;
            dev->num_resolved_drive_pins++;
        }
    }

    /* Params */
    cJSON *params_arr = cJSON_GetObjectItem(dev_json, "params");
    if (params_arr && cJSON_IsArray(params_arr)) {
        cJSON *pd = NULL;
        cJSON_ArrayForEach(pd, params_arr) {
            if (!cJSON_IsObject(pd)) continue;
            cJSON *idx_item = cJSON_GetObjectItem(pd, "index");
            if (!idx_item || !cJSON_IsNumber(idx_item)) continue;
            int pidx = idx_item->valueint;
            if (pidx < 0 || pidx >= GTE_MAX_PARAMS) continue;

            cJSON *def_item = cJSON_GetObjectItem(pd, "default");
            if (def_item && cJSON_IsNumber(def_item)) {
                dev->params[pidx] = (float)def_item->valuedouble;
            }
            if (pidx >= dev->num_params) {
                dev->num_params = pidx + 1;
            }
        }
    }

    /* Parse on_trigger steps — use drive_pin_names so that "set": "echo"
     * resolves to drive_irqs[0], not pin_names[1] */
    cJSON *on_trigger = cJSON_GetObjectItem(dev_json, "on_trigger");
    if (on_trigger && cJSON_IsArray(on_trigger)) {
        dev->num_steps = gte_parse_steps(on_trigger, dev, drive_pin_names,
                                          num_drive_pin_names, dev_json, 0);
    }

    /* Parse end_sequence if present (appended after main steps) */
    cJSON *end_seq = cJSON_GetObjectItem(dev_json, "end_sequence");
    if (end_seq && cJSON_IsArray(end_seq)) {
        dev->num_steps = gte_parse_steps(end_seq, dev, drive_pin_names,
                                          num_drive_pin_names, NULL,
                                          dev->num_steps);
    }

    return true;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int gte_parse_scripts(const char *file_path, GteDevice devices[])
{
    /* Read JSON file */
    FILE *fp = fopen(file_path, "rb");
    if (!fp) {
        fprintf(stderr, "GTPE: failed to open file: %s\n", file_path);
        return -1;
    }

    fseek(fp, 0, SEEK_END);
    long length = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char *content = (char *)malloc(length + 1);
    if (!content) {
        fclose(fp);
        fprintf(stderr, "GTPE: out of memory reading %s\n", file_path);
        return -1;
    }

    size_t read_len = fread(content, 1, length, fp);
    fclose(fp);
    content[read_len] = '\0';

    cJSON *root = cJSON_Parse(content);
    free(content);

    if (!root) {
        fprintf(stderr, "GTPE: failed to parse JSON: %s\n",
                cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown");
        return -1;
    }

    if (!cJSON_IsArray(root)) {
        fprintf(stderr, "GTPE: JSON root is not an array\n");
        cJSON_Delete(root);
        return -1;
    }

    int count = 0;
    cJSON *dev_json = NULL;
    cJSON_ArrayForEach(dev_json, root) {
        if (count >= GTE_MAX_DEVICES) break;
        if (!cJSON_IsObject(dev_json)) continue;

        if (gte_parse_one_device(dev_json, &devices[count])) {
            fprintf(stderr, "GTPE: parsed device '%s' (%d steps, %d params)\n",
                    devices[count].name, devices[count].num_steps,
                    devices[count].num_params);
            count++;
        }
    }

    cJSON_Delete(root);
    return count;
}

void gte_attach(GteDevice *dev, avr_t *avr,
                char trigger_port, int trigger_pin,
                char drive_ports[], int drive_pins[], int num_drive)
{
    if (!dev || !avr) return;
    dev->avr = avr;

    /* Get trigger IRQ via the ioport subsystem.
     * AVR_IOPORT_OUTPUT offset gives us the pin's output IRQ so we can
     * detect when firmware drives the trigger pin. */
    dev->watch_irq = avr_io_getirq(avr,
        AVR_IOCTL_IOPORT_GETIRQ(trigger_port),
        trigger_pin);

    if (dev->watch_irq) {
        avr_irq_register_notify(dev->watch_irq, gte_trigger_notify, dev);
    }

    /* Set up drive IRQs */
    int n = num_drive;
    if (n > GTE_MAX_DRIVE_PINS) n = GTE_MAX_DRIVE_PINS;

    for (int i = 0; i < n; i++) {
        dev->drive_irqs[i] = avr_io_getirq(avr,
            AVR_IOCTL_IOPORT_GETIRQ(drive_ports[i]),
            drive_pins[i]);

        /* Set initial idle state on drive pins */
        if (dev->drive_irqs[i]) {
            avr_raise_irq(dev->drive_irqs[i], dev->idle_values[i] != 0);
        }
    }

    if (n > dev->num_drive_irqs) {
        dev->num_drive_irqs = n;
    }

    fprintf(stderr, "GTPE: attached '%s' (watch=%c%d, drive=%d pins)\n",
            dev->name, trigger_port, trigger_pin, dev->num_drive_irqs);
}

/**
 * Convert an Arduino digital pin number to an AVR port letter and pin index.
 * ATmega328P mapping:
 *   D0-D7   → PORTD 0-7
 *   D8-D13  → PORTB 0-5
 *   D14-D19 → PORTC 0-5  (A0-A5)
 */
static void arduino_pin_to_avr(int dpin, char *port, int *pin)
{
    if (dpin >= 0 && dpin <= 7) {
        *port = 'D'; *pin = dpin;
    } else if (dpin >= 8 && dpin <= 13) {
        *port = 'B'; *pin = dpin - 8;
    } else if (dpin >= 14 && dpin <= 19) {
        *port = 'C'; *pin = dpin - 14;
    } else {
        *port = 'D'; *pin = 0;
        fprintf(stderr, "GTPE: unknown Arduino pin %d, defaulting to D0\n", dpin);
    }
}

void gte_attach_auto(avr_t *avr, GteDevice *dev)
{
    if (!dev || !avr) return;

    char trig_port;
    int trig_pin;
    arduino_pin_to_avr(dev->resolved_watch_pin, &trig_port, &trig_pin);

    char drive_ports[GTE_MAX_DRIVE_PINS];
    int  drive_pins_avr[GTE_MAX_DRIVE_PINS];

    for (int i = 0; i < dev->num_resolved_drive_pins; i++) {
        arduino_pin_to_avr(dev->resolved_drive_pins[i], &drive_ports[i],
                           &drive_pins_avr[i]);
    }

    gte_attach(dev, avr, trig_port, trig_pin,
               drive_ports, drive_pins_avr, dev->num_resolved_drive_pins);
}

void gte_update_param(GteDevice *dev, int param_index, float value)
{
    if (!dev || param_index < 0 || param_index >= GTE_MAX_PARAMS) return;
    dev->params[param_index] = value;
    if (param_index >= dev->num_params) {
        dev->num_params = param_index + 1;
    }
}
