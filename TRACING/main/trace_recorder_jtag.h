#ifndef TRACE_RECORDER_H
#define TRACE_RECORDER_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRACE_EVT_SW_IN  = 1,
    TRACE_EVT_SW_OUT = 2,
    TRACE_EVT_TICK   = 3,
    TRACE_EVT_MARK   = 4,
} trace_evt_type_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint16_t evt;
    uint8_t  core;
    uint8_t  prio;
    uint32_t task_id;
    uint32_t aux;
} trace_evt_t;

void trace_init(void);
void trace_register_task(TaskHandle_t h, const char *name, uint32_t task_id);

void trace_task_switched_in(void);
void trace_task_switched_out(void);
void trace_tick_hook(void);
void trace_user_mark(uint32_t mark);

void trace_flush_task(void *arg);

uint32_t trace_get_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif