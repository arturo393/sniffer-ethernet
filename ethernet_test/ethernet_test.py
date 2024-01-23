import socket
import logging
import sys
import time

logging.basicConfig(filename="serialTest.log", level=logging.DEBUG)

def _transmit_and_receive_tcp(self, address, port, data):
    rt = time.time()
    reply_counter = 0
    exception_message = "CRITICAL - "
    for cmd_name in self.list:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((address, port))
                sock.settimeout(2)
                data_bytes = bytearray.fromhex(cmd_name.query)
                sock.sendall(data_bytes)
                logging.debug("Message retransmitted:")
                logging.debug(f"{data_bytes}")
                logging.debug("Message length: " + str(len(data)))
                logging.debug("-----")
                sock.sendall(data.encode())
                data_received = sock.recv(1024)
                sock.close()
                cmd_name.reply = data_received
                logging.debug("Datos recibidos:")
                logging.debug(data_received)
                reply_counter = reply_counter + 1
        except Exception as e:
            return 0

    rt = str(time.time() - rt)
    self.parameters['rt'] = rt
    return self.parameters

while(True):
    try:
        data = "Hola, Mundo!"
        _transmit_and_receive_tcp('192.168.0.2', 3000, data)
    except Exception as e:
        logging.error(e)
        sys.exit()
