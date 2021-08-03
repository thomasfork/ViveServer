import socket
import struct
import dill

MCAST_GRP = '239.0.0.0' # Administratively scoped IPv4 address space
MCAST_PORT = 5007
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except AttributeError:
    # not available on Windows
    pass
sock.bind((MCAST_GRP, MCAST_PORT))


mreq = struct.pack('4sl', socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)         
                
while True:
  print(dill.loads(sock.recv(1024)))
