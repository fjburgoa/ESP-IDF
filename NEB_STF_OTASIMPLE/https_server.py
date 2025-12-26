import http.server
import ssl

IP = "192.168.0.15"                 #IP de la máquina que da el binario
PORT = 8070

handler = http.server.SimpleHTTPRequestHandler
httpd = http.server.HTTPServer((IP, PORT), handler)

# Crear contexto TLS
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile="ca_cert.pem", keyfile="ca_key.pem")

# Envolver el socket
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

print(f"Serving HTTPS on https://{IP}:{PORT}")
httpd.serve_forever()
