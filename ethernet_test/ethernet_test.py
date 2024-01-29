import socket
import logging
import sys

SNIFFER_SERVER_IP_ADDR = '192.168.60.101'
SNIFFER_SERVER_TCP_PORT = 3000


logging.basicConfig(filename="ethernet_test.log", level=logging.DEBUG)
        
while True:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((SNIFFER_SERVER_IP_ADDR, SNIFFER_SERVER_TCP_PORT))
            #sock.settimeout(2)
            while True:
                data = sock.recv(1024)  # Receive 1024 bytes at a time
                logging.debug('Recibido {!r}'.format(data))
                data = bytearray(data)
                logging.debug("Message retransmitted:")
                sock.sendall(data)
                    
    except Exception as e:
        logging.error(e)
        sys.exit()