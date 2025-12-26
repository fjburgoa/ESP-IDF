#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "dft.h"

//INDICE TAREAS
#define TASK1 0
#define TASK2 1
#define TASK3 2
#define TASK4 3
#define NTASKS TASK4+1

//PERIODOS TAREAS en ms
#define TASK1_T 1
#define TASK2_T 1000
#define TASK3_T 1
#define TASK4_T 100

//GPIO SALIDAS Y ENTRADAS DIGITALES
#define LED4 4
#define LED6 6
#define PULS 0

#define LEDC_MODE   LEDC_LOW_SPEED_MODE

#define EXAMPLE_ADC1_CHAN0      ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_12

#define STACK_SIZE	3*1024	     //N x 1kByte es el tamaño de la piLa   

#define MAX_DATA  1000/TASK1_T

//-----TIMER ESP------------------------
esp_timer_handle_t myTimerESP;   //handler a timer

//-----Semaphore------------------------
SemaphoreHandle_t xSemaphore;    //handler a semáforo

//-----ADC------------------------------
adc_oneshot_unit_handle_t adc1_handle; //handler ADC de medida única
int   adc_raw;                         //medida
int   raw_signal[MAX_DATA] = {0};      //vector 

//-----DFT------------------------------
unsigned int indx  = 0;
int   harmonic_nr[MAX_DATA/2]     = {0}; 
float magnitude[MAX_DATA/2]       = {0};
float relative_weight[MAX_DATA/2] = {0};
float valor_medio  = 0.0;

//----Statistics------------------------
char Buff[512]                    = {0};
unsigned long exec_time[4]        = {0};

//----test signal-----------------------
float frequency                   = 50.0;             //frecuencia 

//----------------------------------------------------------------
//--------------Task1: ADC, toma un dato cada milisegundo
//----------------------------------------------------------------
void vTaskCode1( void * pvParameters )           
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK1_T/portTICK_PERIOD_MS;
      
    unsigned long us1,us2 = 0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
       //toma tiempo
       us1 = esp_timer_get_time();  

       //toma medida
       adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw);
       raw_signal[indx] = adc_raw;
       indx++;       

       //ha leido 1000 datos, inicia task procesado de datos  
       if(indx>MAX_DATA-1)
       {
            //copia datos a signal
            indx = 0;            
            for (int k=0;k<MAX_DATA;k++)
            {
                signal[k].real = (float)(raw_signal[k])/4095;   //parte real
                signal[k].imag = 0.0f;                          //parte imaginaria
            }         

            //start semaforo para Task 3
            xSemaphoreGive(xSemaphore);
       }
       //calcula dT
       us2          = esp_timer_get_time();
       exec_time[TASK1] = (90*exec_time[TASK1] +  10*(us2-us1))/100;

       //Suspende y espera hasta 1ms  
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );     
    }	
}
//----------------------------------------------------------------
//-----------------Task2: FFT, inicia con semáforo  --------------
//----------------------------------------------------------------
void vTaskCode2( void * pvParameters )           //
{
    unsigned long us1,us2 = 0;
    while(1)
    {
       if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
       {
           us1=esp_timer_get_time();           
           //calcula el valor medio
           for (int k=0;k<MAX_DATA;k++){
               valor_medio +=signal[k].real;
           }

           //calcula el valor medio     
           valor_medio = valor_medio/MAX_DATA;   
           
           //resta el valor medio a la serie
           for (int k=0;k<MAX_DATA;k++){
                signal[k].real = signal[k].real-valor_medio;
           }
          
           //calcula la FFT
           fft(signal,MAX_DATA);
           findTopHarmonics(signal, harmonics, 10, weights);        
           //Harmonics(signal,harmonic_nr,magnitude,relative_weight);
           
           //calcula dT
           us2          = esp_timer_get_time();
           exec_time[TASK2] = (90*exec_time[TASK2] +  10*(us2-us1))/100;
       }
    }	
}
//----------------------------------------------------------------
//-----------------Task3: Genera señal prueba  -------------------
//----------------------------------------------------------------
void vTaskCode3( void * pvParameters )           
{
    unsigned long us1,us2 = 0;

    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK3_T/portTICK_PERIOD_MS;

    uint32_t dutycycle = 0;
    float internal_t   = 0.0;

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    
    while(1)
    {      
       //Get time
       us1       = esp_timer_get_time();           
       
       //genera señal portadora
       internal_t += 0.001;
       dutycycle = (unsigned int)(4095*0.5*(1.0 + sin(2*3.1415926*internal_t*frequency)));

       //actualiza duty cycle
       ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, dutycycle);     //copiamos el valor de la lectura ADC al led
       ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);       
     
       //calcula dT
       us2              = esp_timer_get_time();
       exec_time[TASK3] = (90*exec_time[TASK3] +  10*(us2-us1))/100;

       //suspende hasta que haya cumplido el tiempo
       xTaskDelayUntil( &xLastWakeTime, xDelayTicks );
    }	
}

//----------------------------------------------------------------
//-----------------Task4: UART  ----------------------------------
//----------------------------------------------------------------
void vTaskCode4( void * pvParameters )             
{
    TickType_t 		    xLastWakeTime; 
    const TickType_t 	xDelayTicks = TASK4_T/portTICK_PERIOD_MS;

    unsigned long us1,us2 = 0;    

    uint8_t data[16];
    int length = 0;

    char seleccion_menu = 0;

    printf("\x1b[2J");
    printf("0-Menu:\n");
    printf("1-Mostrar armónicos\n");
    printf("2-Cambiar frecuencia señal de prueba +1Hz\n");
    printf("3-Cambiar frecuencia señal de prueba -1Hz\n");
    printf("4-Mostrar carga de procesos\n");
    printf("5-Duración de cada proceso\n");    

    xLastWakeTime = xTaskGetTickCount ();         // Initialise the xLastWakeTime variable with the current time. 
    while(1)
    {
        //Get time
        us1       = esp_timer_get_time();           

        uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length);
        if ((length>0)&&((length<3)))
        {
            length = uart_read_bytes(UART_NUM_0, data, length, 16); 
            seleccion_menu = data[0];

            switch (seleccion_menu)
            {
                case '0':
                    printf("\x1b[2J");
                    printf("0-Menu:\n");
                    printf("1-Mostrar armónicos\n");
                    printf("2-Cambiar frecuencia señal de prueba +1Hz\n");
                    printf("3-Cambiar frecuencia señal de prueba -1Hz\n");
                    printf("4-Mostrar carga de procesos\n");
                    printf("5-Duración de cada proceso\n");
                    break;
                case '1':
                    printf("\x1b[2J");              
                    for (int i = 0; i < 10; ++i)
                        printf("%d Hz - %.1f%%\n", harmonics[i], weights[i]);
                    
                    printf("Pulsa 0+Intro para volver al menu\n");
                    break;
                
                case '2':
                    printf("\x1b[2J");              
                    frequency = frequency +1.0;
                    printf("Nueva frecuencia: %.1f\n",frequency);
                    printf("Pulsa 0+Intro para volver al menu\n");
                    break;
                case '3':
                    printf("\x1b[2J");  
                    frequency = frequency -1.0;
                    printf("Nueva frecuencia: %.1f\n",frequency);
                    printf("Pulsa 0+Intro para volver al menu\n");
                    break;
                case '4':
                    printf("\x1b[2J");  
                    printf("%s ", Buff);  //habría que protegerla
                    printf("Pulsa 0+Intro para volver al menu\n");
                    break;
                case '5':
                    printf("\x1b[2J");  
                    for (int k = 0; k<NTASKS; k++)
                        printf("avg exec time Task %d: %lu us\n", k+1,exec_time[k]);  //habría que protegerla
                    
                    printf("Pulsa 0+Intro para volver al menu\n");
                    break;                    
            }
        }

        //calcula dT
        us2          = esp_timer_get_time();
        exec_time[TASK3] = (90*exec_time[TASK3] +  10*(us2-us1))/100;

        xTaskDelayUntil( &xLastWakeTime, xDelayTicks );
    }
}

//----------------------------------------------------------------
//-----------------Timer ESP  ------------------------------------
//----------------------------------------------------------------
void ESP_TimerCallback (void)
{
    //nothing to do 
}

//----------------------------------------------------------------
//-----------------Task3: Main  ----------------------------------
//----------------------------------------------------------------
void app_main(void) 
{   
    //configura el Timer    
    const esp_timer_create_args_t MyTimerESPConfig = 
    {
        .callback = &ESP_TimerCallback,                //callback            
        .name = "My_Periodic"                          //name    
    };

    esp_timer_create(&MyTimerESPConfig, &myTimerESP);
    esp_timer_start_periodic(myTimerESP, 1000000);   //1 s

    //Configura GPIO
    gpio_set_direction(PULS,GPIO_PULLUP_ENABLE);
    gpio_set_direction(LED4,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED6,GPIO_MODE_OUTPUT);

    //Configura el ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config);

    //Configura la salida PWM (para generar señal de prueba)
    ledc_channel_config_t ledc_channel = {
        .speed_mode       = LEDC_MODE, 	
        .channel          = LEDC_CHANNEL_0,
        .timer_sel        = LEDC_TIMER_0,
        .intr_type        = LEDC_INTR_DISABLE,
        .gpio_num         = 6,   
        .duty             = 0,         // variable de tipo uint32_t y relativa a la resolución del timer
    };
    ledc_channel_config(&ledc_channel);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_12_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 8000,             // 8 kHz de frecuencia
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);    

    // UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver
    #define BUF_SIZE 1024
    uart_driver_install(UART_NUM_0, BUF_SIZE , 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Cola 
    //HandlerCola = xQueueCreate ( 40, sizeof(uint32_t));           //Crea la cola

    //Configura el Semáforo
    xSemaphore = xSemaphoreCreateBinary();                          //semáforo

    //Crea Handlers
    TaskHandle_t xHandle1 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle2 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle3 = NULL;  //Handler a la tarea
    TaskHandle_t xHandle4 = NULL;  //Handler a la tarea

    int ucParameterToPass = 0;     //dummy 

    xTaskCreate( vTaskCode1, "TASK1", STACK_SIZE, &ucParameterToPass, 3, &xHandle1 );  //Prioridad máxima
    xTaskCreate( vTaskCode2, "TASK2", STACK_SIZE, &ucParameterToPass, 1, &xHandle2 );  //Prioridad baja 
    xTaskCreate( vTaskCode3, "TASK3", STACK_SIZE, &ucParameterToPass, 2, &xHandle3 );  //Prioridad media   
    xTaskCreate( vTaskCode4, "TASK4", STACK_SIZE, &ucParameterToPass, 1, &xHandle4 );  //Sin prioridad 

    while (1) 
    {
       vTaskGetRunTimeStats(Buff);
       vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}


 