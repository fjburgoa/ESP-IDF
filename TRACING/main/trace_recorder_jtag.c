#include "trace_recorder_jtag.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_app_trace.h"
#include "esp_err.h"

#define TRACE_RING_SIZE      4096u     // número de eventos
#define TRACE_MAX_TASKS      32u
#define TRACE_FLUSH_CHUNK    64u      // eventos por envío máx

typedef struct {
    TaskHandle_t handle;
    uint32_t task_id;
    char name[20];
} task_map_t;

static trace_evt_t s_ring[TRACE_RING_SIZE];
static volatile uint32_t s_wr = 0;
static volatile uint32_t s_rd = 0;
static volatile uint32_t s_drops = 0;

static task_map_t s_tasks[TRACE_MAX_TASKS];
static uint32_t s_num_tasks = 0;

static inline uint32_t trace_time_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

static inline uint8_t trace_core_id(void)
{
    return (uint8_t)esp_cpu_get_core_id();
}

static inline uint32_t trace_find_task_id(TaskHandle_t h)
{
    for (uint32_t i = 0; i < s_num_tasks; i++) {
        if (s_tasks[i].handle == h) {
            return s_tasks[i].task_id;
        }
    }
    return 0;   // desconocida/idle/no registrada
}

static inline uint8_t trace_task_prio(TaskHandle_t h)
{
    if (h == NULL) return 0;
    return (uint8_t)uxTaskPriorityGet(h);
}

static inline bool trace_ring_push(trace_evt_t e)
{
    uint32_t wr = s_wr;
    uint32_t next = (wr + 1u) % TRACE_RING_SIZE;

    if (next == s_rd) {
        s_drops++;
        return false; // ring lleno
    }

    s_ring[wr] = e;
    __asm__ __volatile__("" ::: "memory");
    s_wr = next;
    return true;
}

static inline bool trace_ring_pop(trace_evt_t *e)
{
    uint32_t rd = s_rd;
    if (rd == s_wr) {
        return false;
    }

    *e = s_ring[rd];
    __asm__ __volatile__("" ::: "memory");
    s_rd = (rd + 1u) % TRACE_RING_SIZE;
    return true;
}

static inline void trace_log_fast(uint16_t evt, TaskHandle_t h, uint32_t aux)
{
    trace_evt_t e;
    e.t_us    = trace_time_us();
    e.evt     = evt;
    e.core    = trace_core_id();
    e.prio    = trace_task_prio(h);
    e.task_id = trace_find_task_id(h);
    e.aux     = aux;
    (void)trace_ring_push(e);
}

void trace_init(void)
{
    s_wr = 0;
    s_rd = 0;
    s_drops = 0;
    s_num_tasks = 0;
    memset(s_ring, 0, sizeof(s_ring));
    memset(s_tasks, 0, sizeof(s_tasks));
}

void trace_register_task(TaskHandle_t h, const char *name, uint32_t task_id)
{
    if ((h == NULL) || (s_num_tasks >= TRACE_MAX_TASKS)) {
        return;
    }

    s_tasks[s_num_tasks].handle = h;
    s_tasks[s_num_tasks].task_id = task_id;
    strncpy(s_tasks[s_num_tasks].name, name ? name : "?", sizeof(s_tasks[s_num_tasks].name) - 1u);
    s_tasks[s_num_tasks].name[sizeof(s_tasks[s_num_tasks].name) - 1u] = '\0';
    s_num_tasks++;
}

void trace_task_switched_in(void)
{
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log_fast(TRACE_EVT_SW_IN, h, 0);
}

void trace_task_switched_out(void)
{
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log_fast(TRACE_EVT_SW_OUT, h, 0);
}

//------------------------------------------------------------------------
void vApplicationTickHook(void) 
{    
    //trace_tick_hook(); //wrapper
}

void trace_tick_hook(void)
{
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log_fast(TRACE_EVT_TICK, h, 0);
}

void trace_user_mark(uint32_t mark)
{
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log_fast(TRACE_EVT_MARK, h, mark);
}

uint32_t trace_get_drop_count(void)
{
    return s_drops;
}

void trace_flush_task(void *arg)
{
    (void)arg;

    trace_evt_t chunk[TRACE_FLUSH_CHUNK];

    while (1) {
        uint32_t n = 0;
        while (n < TRACE_FLUSH_CHUNK && trace_ring_pop(&chunk[n])) {
            n++;
        }

        if (n > 0) 
        {
            esp_err_t err ;
            const uint32_t bytes = n * sizeof(trace_evt_t);

            // Opción A: simple
            //esp_err_t err = esp_apptrace_write(chunk, bytes, 0);
            //esp_err_t err = esp_apptrace_write(ESP_APPTRACE_DEST_JTAG, chunk, bytes, 0);

            // Opción B: más fina, cero copia en el buffer apptrace
            uint8_t *p = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_JTAG, bytes, 0);
            if (p) {
                memcpy(p, chunk, bytes);
                err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_JTAG, p, 0);
            } else {
                err = ESP_ERR_NO_MEM;
            }

            (void)err;
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}