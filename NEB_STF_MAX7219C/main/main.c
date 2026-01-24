#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define PIN_CLK 6
#define PIN_CS  5
#define PIN_DIN 4

//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------

static const uint8_t digits[10][8] = {
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00},  //0
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00},  //1 
    {0x3E,0x63,0x03,0x1E,0x30,0x60,0x7F,0x00},  //2 
    {0x3E,0x63,0x03,0x1E,0x03,0x63,0x3E,0x00},  //3
    {0x06,0x0E,0x1E,0x36,0x7F,0x06,0x06,0x00},  //4
    {0x7F,0x60,0x7E,0x03,0x03,0x63,0x3E,0x00},  //5 
    {0x1E,0x30,0x60,0x7E,0x63,0x63,0x3E,0x00},  //6 
    {0x7F,0x03,0x06,0x0C,0x18,0x18,0x18,0x00},  //7
    {0x3E,0x63,0x63,0x3E,0x63,0x63,0x3E,0x00},  //8
    {0x3E,0x63,0x63,0x3F,0x03,0x06,0x3C,0x00}   //9 
};

static const uint8_t letters_upper[26][8] = {
    {0x00,0x18,0x24,0x42,0x7E,0x42,0x42,0x00}, // A
    {0x00,0x7C,0x42,0x7C,0x42,0x42,0x7C,0x00}, // B
    {0x00,0x3C,0x42,0x40,0x40,0x42,0x3C,0x00}, // C
    {0x00,0x78,0x44,0x42,0x42,0x44,0x78,0x00}, // D
    {0x00,0x7E,0x40,0x7C,0x40,0x40,0x7E,0x00}, // E
    {0x00,0x7E,0x40,0x7C,0x40,0x40,0x40,0x00}, // F
    {0x00,0x3C,0x42,0x40,0x4E,0x42,0x3C,0x00}, // G
    {0x00,0x42,0x42,0x7E,0x42,0x42,0x42,0x00}, // H
    {0x00,0x3C,0x18,0x18,0x18,0x18,0x3C,0x00}, // I
    {0x00,0x1E,0x04,0x04,0x04,0x44,0x38,0x00}, // J
    {0x00,0x42,0x44,0x78,0x44,0x42,0x42,0x00}, // K
    {0x00,0x40,0x40,0x40,0x40,0x40,0x7E,0x00}, // L
    {0x00,0x42,0x66,0x5A,0x42,0x42,0x42,0x00}, // M
    {0x00,0x42,0x62,0x52,0x4A,0x46,0x42,0x00}, // N
    {0x00,0x3C,0x42,0x42,0x42,0x42,0x3C,0x00}, // O
    {0x00,0x7C,0x42,0x42,0x7C,0x40,0x40,0x00}, // P
    {0x00,0x3C,0x42,0x42,0x42,0x4A,0x3C,0x02}, // Q
    {0x00,0x7C,0x42,0x42,0x7C,0x44,0x42,0x00}, // R
    {0x00,0x3C,0x40,0x3C,0x02,0x42,0x3C,0x00}, // S
    {0x00,0x7E,0x18,0x18,0x18,0x18,0x18,0x00}, // T
    {0x00,0x42,0x42,0x42,0x42,0x42,0x3C,0x00}, // U
    {0x00,0x42,0x42,0x42,0x42,0x24,0x18,0x00}, // V
    {0x00,0x42,0x42,0x42,0x5A,0x66,0x42,0x00}, // W
    {0x00,0x42,0x24,0x18,0x18,0x24,0x42,0x00}, // X
    {0x00,0x42,0x24,0x18,0x18,0x18,0x18,0x00}, // Y
    {0x00,0x7E,0x04,0x08,0x10,0x20,0x7E,0x00}  // Z
};

static const uint8_t letters_minor[26][8] = {
    {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00}, // a
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // b
    {0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00}, // c
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, // d
    {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, // e
    {0x1C,0x36,0x30,0x7C,0x30,0x30,0x30,0x00}, // f
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x3C}, // g
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // h
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // i
    {0x06,0x00,0x06,0x06,0x06,0x66,0x66,0x3C}, // j
    {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00}, // k
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // l
    {0x00,0x00,0x66,0xFF,0xDB,0xDB,0xDB,0x00}, // m
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, // n
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // o
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // p
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, // q
    {0x00,0x00,0x6C,0x76,0x60,0x60,0x60,0x00}, // r
    {0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00}, // s
    {0x30,0x30,0x7C,0x30,0x30,0x36,0x1C,0x00}, // t
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, // u
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // v
    {0x00,0x00,0x63,0x63,0x6B,0x7F,0x36,0x00}, // w
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // x
    {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C}, // y
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}  // z
};

static const uint8_t letter_enye_minor[8] = {
    0x00,0x3C,0x00,0x7C,0x66,0x66,0x66,0x00    // ñ 
};

static const uint8_t letter_enye_upper[8] = {
    0x3C,0x00,0x42,0x62,0x52,0x4A,0x46,0x00    // Ñ  
};

static const uint8_t letter_a_acute[8] = {
    0x04,0x08,0x3C,0x06,0x3E,0x66,0x3E,0x00    // á
};

static const uint8_t letter_e_acute[8] = {
    0x04,0x08,0x3C,0x66,0x7E,0x60,0x3C,0x00    // é
};

static const uint8_t letter_i_acute[8] = {
    0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00    // í = i (no cabe)
};

static const uint8_t letter_o_acute[8] = {
    0x04,0x08,0x3C,0x66,0x66,0x66,0x3C,0x00    // ó
};

static const uint8_t letter_u_acute[8] = {
    0x04,0x08,0x66,0x66,0x66,0x66,0x3E,0x00    // ú
};

static const uint8_t space[8] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00    //" " - Espacio
};

static const uint8_t symbols[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // .  punto
    {0x00,0x00,0x00,0x00,0x18,0x18,0x30,0x00}, // ,  coma
    {0x00,0x00,0x18,0x18,0x00,0x18,0x18,0x00}, // :  dos puntos
    {0x00,0x00,0x18,0x18,0x00,0x18,0x18,0x30}, // ;  punto y coma
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // -  guion
    {0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x00}, // /  barra lateral
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00}, // _  barra baja
    {0x00,0x18,0x18,0x18,0x18,0x00,0x18,0x00}, // !  exclamación
    {0x00,0x3C,0x42,0x04,0x08,0x00,0x08,0x00}, // ?  interrogación
    {0x3C,0x42,0x5A,0x5A,0x5E,0x40,0x3C,0x00}, // @
    {0x04,0x08,0x10,0x20,0x10,0x08,0x04,0x00}, // <
    {0x20,0x10,0x08,0x04,0x08,0x10,0x20,0x00}, // >
    {0x1C,0x22,0x70,0x20,0x70,0x22,0x1C,0x00}, // €
    {0x62,0x64,0x08,0x10,0x26,0x46,0x00,0x00}, // %
    {0x08,0x10,0x20,0x20,0x20,0x10,0x08,0x00}, // (
    {0x10,0x08,0x04,0x04,0x04,0x08,0x10,0x00}, // ) 
    {0x14,0x14,0x7F,0x14,0x7F,0x14,0x14,0x00}  // #  
};
 
// Enumerado para los símbolos
enum {
    SYM_DOT = 0,
    SYM_COMMA,
    SYM_COLON,
    SYM_SEMICOLON,
    SYM_MINUS,
    SYM_SLASH,
    SYM_UNDERSCORE,
    SYM_EXCL,
    SYM_QUESTION,
    SYM_AT,
    SYM_MINOR,
    SYM_MORE,
    SYM_EURO,
    SYM_PERCENTAGE,
    SYM_OPEN_PARENTESIS,
    SYM_CLOSE_PARENTESIS,
    SYM_HASTAG
};
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------

#define NDISPLAY 10    //Número de displays MAX7219C en cascada

//--------------------------------------------------------------------
static void max7219_send(uint8_t addr, uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_CLK, 0);
        gpio_set_level(PIN_DIN, (addr >> i) & 1);
        gpio_set_level(PIN_CLK, 1);
    }
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(PIN_CLK, 0);
        gpio_set_level(PIN_DIN, (data >> i) & 1);
        gpio_set_level(PIN_CLK, 1);
    }
}
//--------------------------------------------------------------------
static void max7219_init(void)
{
    gpio_set_level(PIN_CS, 0);
    for (int i = 0; i < NDISPLAY; i++) max7219_send(0x0F, 0x00); // test off
    gpio_set_level(PIN_CS, 1);

    gpio_set_level(PIN_CS, 0);
    for (int i = 0; i < NDISPLAY; i++) max7219_send(0x09, 0x00); // no decode
    gpio_set_level(PIN_CS, 1);

    gpio_set_level(PIN_CS, 0);
    for (int i = 0; i < NDISPLAY; i++) max7219_send(0x0B, 0x07); // 8 filas
    gpio_set_level(PIN_CS, 1);

    gpio_set_level(PIN_CS, 0);
    for (int i = 0; i < NDISPLAY; i++) max7219_send(0x0A, 0x0F); // brillo
    gpio_set_level(PIN_CS, 1);

    gpio_set_level(PIN_CS, 0);
    for (int i = 0; i < NDISPLAY; i++) max7219_send(0x0C, 0x01); // on
    gpio_set_level(PIN_CS, 1);
}
//----------------------------------------------------------------------
void update_N_digits(uint8_t* M, int lcadena)
{
    //esta función rota una columna todo la matriz M. Lo hace desplazando a la izquierda
    //y teniendo en cuenta el carry del de la derecha.
 
    uint8_t C[8] ={0};
    
    //Copia el primero (más a la izquierda) en una variable para luego rescatarlo y hacer el scroll por la derecha
    for (int j = 0;j<8;j++)
    {
        C[j] = M[j];
    };
    
    for (int k = 0; k< lcadena-1; k++)
    {
        for (int j = k*8; j<(k+1)*8 ; j++)
        {
            M[j]=M[j]<<1;                  //desplaza a la izquierda

            //ten en cuenta que si la linea siguiente tiene un '1' en la posición más a la izquierda,
            //al desplazarlo a la izquierda, se sumará al digito actual. Es lo que hacemos aquí. Si el 
            //bit de la izquierda (0x80) es '1', entonces se lo sumamos a la línea del dígito actual
            uint8_t  c = ((M[j+8]) & 0x80);
            if (c)
                M[j]+=1;
        }        
    }

    //en el último dígito, hacemos el scroll con el primero, rescatando el vector C
    int k = lcadena-1;
    for (int j = k*8; j<(k+1)*8 ; j++)
    {
        M[j]=M[j]<<1;                       //desplaza a la izquierda    
        uint8_t  c = ((C[j-k*8]) & 0x80);
        if (c)
            M[j]+=1;
    }        
}
//---------------------------------------------------------------------------------
void init_M(const char *s, uint8_t *M)
{
    //Esfa función rellena la matriz M descodificando la cadena s en las lineas correspondientes del dígito
    //ya sean (letra mayúscula, minúscula, número, espacio o símbolo especial)
    size_t k   = 0;
    size_t out = 0;

    while (s[k])
    {
        const uint8_t *glyph = space;
        
        if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0xB1){
            glyph = letter_enye_minor;          // UTF-8 ñ es 0xC3 0xB1, por eso suma +2 
            k += 2;
        }
        else if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0x91) {
            glyph = letter_enye_upper;          // UTF-8 ñ es 0xC3 0x91, por eso suma +2  
            k += 2;
        }
        else if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0xA1) {
            glyph = letter_a_acute;             //á 
            k += 2;
        }
        else if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0xA9) {
            glyph = letter_e_acute;             //é
            k += 2;
        }
        else if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0xAD) {
            glyph = letter_i_acute;             //í
            k += 2;
        }
        else if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0xB3) {
            glyph = letter_o_acute;             //ó 
            k += 2;
        }
        else if ((uint8_t)s[k] == 0xC3 && (uint8_t)s[k+1] == 0xBA) {
            glyph = letter_u_acute;             //ú
            k += 2;
        }
        else {
            char ch = s[k++];

            if (ch >= '0' && ch <= '9')         
                glyph = digits[ch - '0'];                      //Número
            else if (ch >= 'a' && ch <= 'z')
                glyph = letters_minor[ch - 'a'];               //letra minúscula
            else if (ch >= 'A' && ch <= 'Z')
                glyph = letters_upper[ch - 'A'];               //letra mayúscula
            else if (ch == ' ')
                glyph = space;                                 //espacio 
            //símbolos    
            else if (ch == '.')  glyph = symbols[SYM_DOT];                      
            else if (ch == ',')  glyph = symbols[SYM_COMMA];
            else if (ch == ':')  glyph = symbols[SYM_COLON];
            else if (ch == ';')  glyph = symbols[SYM_SEMICOLON];
            else if (ch == '-')  glyph = symbols[SYM_MINUS];
            else if (ch == '/')  glyph = symbols[SYM_SLASH];
            else if (ch == '_')  glyph = symbols[SYM_UNDERSCORE];
            else if (ch == '!')  glyph = symbols[SYM_EXCL];
            else if (ch == '?')  glyph = symbols[SYM_QUESTION];
            else if (ch == '@')  glyph = symbols[SYM_AT];
            else if (ch == '<')  glyph = symbols[SYM_MINOR];
            else if (ch == '>')  glyph = symbols[SYM_MORE];
            else if ((uint8_t)ch == 0xE2 && (uint8_t)s[k] == 0x82 && (uint8_t)s[k+1] == 0xAC) 
            {
                glyph = symbols[SYM_EURO];   // €
                k += 2;
            }
            else if (ch == '%')  glyph = symbols[SYM_PERCENTAGE];
            else if (ch == '(')  glyph = symbols[SYM_OPEN_PARENTESIS];                                                                                
            else if (ch == ')')  glyph = symbols[SYM_CLOSE_PARENTESIS];
            else if (ch == '#')  glyph = symbols[SYM_HASTAG];                
            else
                glyph = space;                                   // fallback seguro 
        }

        for (int row = 0; row < 8; row++)                        // copia a Matriz M
            M[out*8 + row] = glyph[row];

        out++;
    }
}
//-------------------------------------------------------------------------
void display_N_digits(uint8_t *M)
{
    //esta función envía a los displays (tantos como NDISPLAY)
    for (int row = 0; row < 8; row++) 
    {
        gpio_set_level(PIN_CS, 0);
        for (int display=0;display<NDISPLAY; display++)
            max7219_send(row + 1, M[row+8*display]);  //envía las líneas empezando por el display más a la izq
                                                      //primero la primera línea de todos los displays, luego la segunda 
                                                      //de todos los displays, y así hasta la última. 
        gpio_set_level(PIN_CS, 1);
    }
}
//-------------------------------------------------------------------------
void app_main(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<PIN_CLK) | (1ULL<<PIN_DIN) | (1ULL<<PIN_CS)
    };
    gpio_config(&io_conf);                 //Configura GPIOx
    
    max7219_init();                        //Inicializa MAX7219 

    //Cadena de texto a mostrar
    char cadena[] = "1234567890 abcdefghijklmnñopqrstuvwxyz ABCDEFGHIJKLMNÑOPQRSTUVWXYZ .,:;-/_!?@ áéíóú ";
    //char cadena[] = "fjburgoa@gmail.com  .,:;-/_!?@<>€%()#    ";
    //char cadena[] = "El ESP32-S3. El microcontrolador más versátil que un camión que tendré. Yúhu!   ";
    
    uint8_t *M = (uint8_t *)malloc(strlen(cadena)*8);    //reserva memoria

    int lcadena =  strlen(cadena)-1;       //calcula la longitud de la cadena 
                                           //si la cadena tuviese caracteres UTF de doble dígito, como ñ o á
                                           //la longitud de cadena no coincidiría con la de caracteres! 

    init_M(cadena,M);                      //rellena M con las líneas correspondientes de cadena
    display_N_digits(M);                   //Muestra por pantalla
    vTaskDelay(pdMS_TO_TICKS(1000));       //delay de 1 segundo

    while (1) 
    {
        update_N_digits(M,lcadena);     //rota a la izquierda
        display_N_digits(M);            //Muestra por pantalla
        vTaskDelay(pdMS_TO_TICKS(70));  //delay de 80 ms
        
        //free(M);                      //no olvidarse de liberar M
    }
}