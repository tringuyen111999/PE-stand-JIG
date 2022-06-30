import socket
import time
import subprocess

HOST = '192.168.110.16' # Server IP or Hostname
PORT = 12345 # Pick an open Port (1000+ recommended), must match the client sport

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')
#managing error exception
try:
	s.bind((HOST, PORT))
except socket.error:
	print ('failed connect ')

# s.bind((HOST, PORT))
s.listen(5)
print('Socket awaiting messages')
(conn, addr) = s.accept()
print('Connected')
time.sleep(10)
# awaiting for message
while 1:
    data = conn.recv(1024).decode('utf-8')
    print ('I sent a message back in response to: ', data)
    reply = ''

    # process your message
    if data == 'start':
        reply = 'testing'
    # elif data == 'This is important':
    #     reply = 'OK, I have done the important thing you have asked me!'
    # #and so on and on until...
    # elif data == 'quit':
    #     conn.send('Terminating')
    #     break
    else:
        reply = 'Unknown command'
    # Sending reply
    conn.send(reply.encode('utf-8'))
    time.sleep(1)
    proc = subprocess.Popen(["iperf3 -s"], stdout=subprocess.PIPE, shell=True)

# conn.close() # Close connections 