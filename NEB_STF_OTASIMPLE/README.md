//Este es un ejemplo que ejecta un código y el ESP32s3 se va a buscar un binario "Ejemplo01.bin" a 
//un servidor https que está ejecutado en 192.168.0.15 con python a través del script https_server.py
//
//Es necesario generar el certificado correcto antes de empezar.



//1-descarga openssl
//https://slproweb.com/products/Win32OpenSSL.html

//2-Actualiza el certificado con:
//openssl req -x509 -newkey rsa:2048 -keyout ca_key.pem -out ca_cert.pem -days 365 -nodes
// cuando pida CN. poner la IP de la máquina que va a tener el binario Ejemplo01.bin, por ejemplo 192.168.0.15

//3-En el programa actualizar la dirección y el nombre ..url =  "https://192.168.0.15:8070/Ejemplo01.bin", --> compilar y flashear

//Para lanzar el servidor con python desde la máquina 192.168.0.15:
//python https_server.py

//probar antes desde un navegador: https://192.168.0.15:8070

//cuando se conecte en el servidor veremos: 
//C:\Users\fjbur\OneDrive\Documentos\ESP_programs\OTA\simple_ota_example>python https_server.py
//Serving HTTPS on https://192.168.0.15:8070
//192.168.0.15 - - [25/Dec/2025 20:23:02] "GET /Ejemplo01.bin HTTP/1.1" 200 -

//Tool (no firefox): https://thelastoutpostworkshop.github.io/microcontroller_devkit/espconnect/