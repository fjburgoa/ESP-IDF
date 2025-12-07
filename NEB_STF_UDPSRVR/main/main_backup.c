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

#define PORT 3333

/*
Arranca WiFi.
Crea socket UDP en puerto 3333.
Espera datagramas.
Cada datagrama → imprime → responde eco.
Si hay error → cierra socket → lo recrea.
*/

void app_main(void)
{
    char rx_buffer[128];             //buffer de entrada
    char addr_str[128];              //Direcc. IP en formato char
    int addr_family = (int)AF_INET;  //IPv4 
    int ip_protocol = 0;            
    struct sockaddr_in6 dest_addr;

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    example_connect();

    while (1) 
    {
        //configura el socket
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;  //protocolo base   

        //crea el socket
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            printf("No se ha podido crear el socket, err: %d\n", errno);
            break;
        }
        else
        {
            printf("Socket creado\n");
        }
       

        // Define el timeout para la función recvfrom()
/*        
        struct timeval timeout;
        timeout.tv_sec  = 10;  //Si en 10 segundos no llega nada, recvfrom devolverá error (EAGAIN o EWOULDBLOCK).
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
*/
        //Asocia el socket al puerto 3333
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            printf("Socket unable to bind: errno %d", errno);
        }
        else
        {
            printf("Socket bound, port %d", PORT);
        }
        
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        while (1) 
        {

            printf("Waiting for data\n");

            int len = recvfrom(sock, 
                               rx_buffer, 
                               sizeof(rx_buffer) - 1, 
                               0, 
                               (struct sockaddr *)&source_addr, 
                               &socklen);

            // Error occurred during receiving
            if (len < 0) {
                printf("recvfrom failed: errno %d\n", errno);
                break;
            }
            else 
            {
                // Dato recibido
                // Obtener la dirección IP del cliente
                if (source_addr.ss_family == PF_INET) 
                {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } 
                else if (source_addr.ss_family == PF_INET6) 
                {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                printf("Received %d bytes from %s:", len, addr_str);
                printf("%s", rx_buffer);

                //enviamos el ECO al cliente    
                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) 
                {
                    printf("Error occurred during sending, err: %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            printf("Cerrando el socket y reiniciando...");
            shutdown(sock, 0);
            close(sock);
        }
    }
}
