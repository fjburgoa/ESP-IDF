#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_timer.h"

#define PORT 3333

void app_main(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    example_connect();

    //----------------------------- DESTINO TX -----------------------------
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.0.15");   // IP destino

    //------------------------------ SOCKET --------------------------------
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (sock < 0) {
        printf("No se ha podido crear el socket, err: %d\n", errno);
        return;
    }
    printf("Socket creado\n");

    //------------------------------ BIND RX -------------------------------
    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(PORT);       // Escucha en el mismo puerto
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int err = bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
    if (err < 0) {
        printf("Error en bind(): %d\n", errno);
        return;
    }

    //---------------------------- TIMEOUT RX ------------------------------
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;   // 20 ms → recvfrom() NO bloquea el bucle
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    //---------------------------- LOOP PRINCIPAL ---------------------------
    int64_t dT = 0;

    while (1)
    {
        //------------------- TRANSMISIÓN (sendto) -------------------
        char mensaje[64] = {0};
        sprintf(mensaje,"dt=%llu us.", dT);
        int len = strlen(mensaje);

        int64_t t1 = esp_timer_get_time();
        err = sendto(sock, mensaje, len, 0,
                     (struct sockaddr*)&dest_addr,
                     sizeof(dest_addr));

        if (err < 0)
            printf("sendto falló: errno=%d\n", errno);

        //---------------------- RECEPCIÓN (recvfrom) -----------------------
        char rx_buffer[128];
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);

        int rxlen = recvfrom(sock, rx_buffer, sizeof(rx_buffer)-1, 0,
                             (struct sockaddr *)&source_addr, &socklen);

        if (rxlen > 0) {
            rx_buffer[rxlen] = 0;
            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &source_addr.sin_addr, ip_str, sizeof(ip_str));

            printf("RX desde %s: %s\n", ip_str, rx_buffer);
        }
        // Si rxlen < 0 por timeout, no pasa nada → se ignora.

        int64_t t2 = esp_timer_get_time();
        dT = t2 - t1;


        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
