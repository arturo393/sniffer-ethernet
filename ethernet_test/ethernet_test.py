import socket
import logging
import sys
import time

SNIFFER_SERVER_IP_ADDR = '192.168.60.101'
SNIFFER_SERVER_TCP_PORT = 3000


logging.basicConfig(
    filename="ethernet_test.log",
    level=logging.DEBUG,
    format="%(asctime)s - %(levelname)s - %(message)s"  # Add timestamp to log format
)    
while True:
    logging.debug(f"Trying to connect to {SNIFFER_SERVER_IP_ADDR} in port {SNIFFER_SERVER_TCP_PORT}")
    try:
          with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((SNIFFER_SERVER_IP_ADDR, SNIFFER_SERVER_TCP_PORT))
            logging.debug("Connection stablished:")
            while True:
                try:
                    logging.debug("Waiting for data:")
                    data = sock.recv(1024)  # Receive 1024 bytes at a time
                    logging.debug('Recibido {!r}'.format(data))
                    data = bytearray(data)
                    logging.debug("Message retransmitted:")
                    sock.sendall(data)

                except socket.timeout:
                    logging.warning("Socket timeout, reconnecting...")
                    break  # Exit inner loop to re-establish connection

                except ConnectionResetError:
                    logging.warning("Connection reset by peer, reconnecting...")
                    break  # Exit inner loop to re-establish connection

    except Exception as e:
        logging.error(f"Connection error: {e}")
        time.sleep(5)  # Wait before retrying