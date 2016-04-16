import socket

UDP_IP = "35.2.36.106"
UDP_PORT = 5005
MESSAGE = "Hello, World!"


sock = None

def init ():
  sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
  return sock 

def send_message (sock, msg):
  sock.sendto(msg, (UDP_IP, UDP_PORT))
