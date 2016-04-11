import socket

UDP_IP = "35.2.127.133"
UDP_PORT = 5005
MESSAGE = "Hello, World!"


sock = None

def init ():
  sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
 

def send_message (msg):
  sock.sendto(msg, (UDP_IP, UDP_PORT))
