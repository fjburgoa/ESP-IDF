#include "trace_recorder.h"
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_cpu.h"

#define TRACE_BUF_SIZE   2048    //máximo número de elementos en el buffer
#define TRACE_MAX_TASKS    32    //maximo número de tareas

//La estructura task_map_t almacena los datos básicos de la tarea: Handle, ID, Nombre
typedef struct 
{
    TaskHandle_t handle;
    uint32_t task_id;
    char name[20];
} task_map_t;

static volatile uint32_t wr_idx = 0;           //indice a los eventos almacenados
static trace_evt_t trace_buf[TRACE_BUF_SIZE];  
static task_map_t  task_map[TRACE_MAX_TASKS];  //vector de estructuras
static uint32_t num_tasks = 0;

/*
funciones static inline: 

define una función que el compilador intenta insertar directamente 
en el lugar de la llamada (inline) para mejorar el rendimiento, 
limitando su visibilidad a un solo archivo fuente (static)

*/

//------------------------------------------------------------------------
void trace_init(void)
{
    memset((void*)trace_buf, 0, sizeof(trace_buf));   //vacía e inicializa el vector
    memset((void*)task_map, 0, sizeof(task_map));     //vacía e inicializa el vector  
}


//----------busca la tarea según su ID -----------------------------------
static inline uint32_t trace_find_task_id(TaskHandle_t h)
{
    for (uint32_t i = 0; i < num_tasks; i++) {
        if (task_map[i].handle == h) {
            return task_map[i].task_id;
        }
    }
    return 0; // 0 = desconocida / idle / no registrada
}
//--------- coge el tiempo en microsegundos ------------------------------
static inline uint32_t trace_time_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

//----------coge el core en uso ------------------------------------------
static inline uint8_t trace_core_id(void)
{
    return (uint8_t)esp_cpu_get_core_id();
}

//-----------coge la prioridad de la tarea -------------------------------
static inline uint8_t trace_get_task_prio(TaskHandle_t h)
{
    if (h == NULL) return 0;
    return (uint8_t)uxTaskPriorityGet(h);
}

//------------------------------------------------------------------------
static inline void trace_log(trace_evt_type_t evt, TaskHandle_t h, uint32_t aux)
{
    uint32_t i = wr_idx;
    if (i >= TRACE_BUF_SIZE) {
        return; // buffer lleno, política simple: dejar de registrar
    }

    trace_buf[i].t_us    = trace_time_us();
    trace_buf[i].evt     = (uint16_t)evt;
    trace_buf[i].core    = trace_core_id();
    trace_buf[i].prio    = trace_get_task_prio(h);
    trace_buf[i].task_id = trace_find_task_id(h);
    trace_buf[i].aux     = aux;

    wr_idx = i + 1;
}

//------------------------------------------------------------------------
void trace_register_task(TaskHandle_t h, const char *name, uint32_t task_id)
{
    if ((h == NULL) || (num_tasks >= TRACE_MAX_TASKS)) return;

    task_map[num_tasks].handle  = h;
    task_map[num_tasks].task_id = task_id;
    strncpy(task_map[num_tasks].name, name ? name : "?", sizeof(task_map[num_tasks].name)-1);
    task_map[num_tasks].name[sizeof(task_map[num_tasks].name)-1] = '\0';
    num_tasks++;
}

//------------------------------------------------------------------------
void trace_task_switched_in(void)
{
    //cada vez que hay un evento de arranque de una tarea, se llama a esta función 
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log(TRACE_EVT_SW_IN, h, 0);  
}

//------------------------------------------------------------------------
void trace_task_switched_out(void)
{
    //cada vez que hay un evento de parada de una tarea, se llama a esta función
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log(TRACE_EVT_SW_OUT, h, 0);
}

//------------------------------------------------------------------------
void trace_tick_hook(void)
{
    //tick del SO. (definido en el KERNEL - normalmente 1000 = 1ms)
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    trace_log(TRACE_EVT_TICK, h, 0);
}

//------------------------------------------------------------------------
void trace_isr_enter(void)
{
    //cada vez que hay se entra en una ISR:
    trace_log(TRACE_EVT_ISR_IN, NULL, 0);
}

//------------------------------------------------------------------------
void trace_isr_exit(void)
{
    //cada vez que hay se sale de una ISR:
    trace_log(TRACE_EVT_ISR_OUT, NULL, 0);
}

//------------------------------------------------------------------------
void trace_user_mark(uint32_t mark)
{
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    //cada vez que el usuario marca un evento al SO 
    //esto lo podríamos hacer indicando     trace_user_mark(x); (x ej 1,2,..,100)
    trace_log(TRACE_EVT_MARK, h, mark);
}

//------------------------------------------------------------------------
void vApplicationTickHook(void) 
{    
    trace_tick_hook(); //wrapper
}


//------------------------------------------------------------------------
//----------------muestra por terminal serie -----------------------------
//------------------------------------------------------------------------
void trace_dump_csv(void)
{
    printf("idx,t_us,evt,core,prio,task_id,aux\n");
    for (uint32_t i = 0; i < wr_idx; i++) {
        printf("%lu,%lu,%u,%u,%u,%lu,%lu\n",
               (unsigned long)i,                     //indice en la lista
               (unsigned long)trace_buf[i].t_us,     //timestamp us
               (unsigned)trace_buf[i].evt,           //evento 
               (unsigned)trace_buf[i].core,          //core
               (unsigned)trace_buf[i].prio,          //prioridad
               (unsigned long)trace_buf[i].task_id,  //tarea
               (unsigned long)trace_buf[i].aux);     //otros 
    }
}

