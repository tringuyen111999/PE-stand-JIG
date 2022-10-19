import socket
import time
import subprocess
import os
import sys
from signal import signal, SIGPIPE, SIG_DFL
signal(SIGPIPE,SIG_DFL) 


HOST = '192.168.110.20' # Server IP or Hostname
PORT = 1234 # Pick an open Port (1000+ recommended), must match the client sport

while True:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(b"Hello, world")
            data = s.recv(1024)
        print(f"Received {data!r}")
    except Exception: pass
    proc = subprocess.Popen(["iperf3 -s"], stdout=subprocess.PIPE, shell=True)
    time.sleep(5)

    
# awaiting for message
# while 1:
    # try:
    #     data = conn.recv(1024).decode('utf-8')
    #     print ('I sent a message back in response to: ', data)
    #     reply = ''

    #     # process your message
    #     if data == 'start':
    #         reply = 'testing'
    #         conn.send(reply.encode('utf-8'))
    #         time.sleep(1)
    #         proc = subprocess.Popen(["iperf3 -s"], stdout=subprocess.PIPE, shell=True)
    #     else:
    #         reply = 'Unknown command'
    #     # print(reply)
    #     # Sending reply
    # except BrokenPipeError or IOError: pass

    # proc = subprocess.Popen(["iperf3 -s"], stdout=subprocess.PIPE, shell=True)
    # time.sleep(1)
    