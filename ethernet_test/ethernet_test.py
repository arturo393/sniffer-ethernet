import socket
import logging

logging.basicConfig(filename="ethernet_test.log", level=logging.DEBUG)

# Crea un objeto socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
logging.debug('socket ready {!r}')
# Conecta el socket a la dirección IP y el puerto del servidor
server_address = ('192.168.60.101', 3000)
sock.connect(server_address)

# Envía los datos al servidor
message = 'Hola, mundo!'
sock.sendall(message.encode())

# Espera la respuesta del servidor
data = sock.recv(1024)
logging.debug('Recibido {!r}'.format(data))

# Cierra la conexión
sock.close()

 