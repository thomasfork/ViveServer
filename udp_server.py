import socket
import time
import dill
from dataclasses import dataclass, field
import numpy as np

from barc3d.pytypes import PythonMsg, OrientationQuaternion
from utils import get_hostname


MULTICAST_TTL = 1 # maximum number of multicast hops
SOCKET_RETRY_INTERVAL = 1
VR_RETRY_INTERVAL = 1

@dataclass
class UDPConfig(PythonMsg):
    host: str = field(default = '239.0.0.0') # Administratively scoped IPv4 address space
    port: int = field(default = 5007)
    
class UDPServer():
    def __init__(self, config:UDPConfig = UDPConfig()):
        self.config = config
            
        self.vr = None
        self.socket = None
        return
    
    
    
    def step(self):
        if not self.vr:
            self.connect_vr()
        if not self.socket: 
            self.connect_socket()
        
        if self.vr and self.socket:
            if self.read_vr():
                self.write_multicast()
        return
        
        
    
    
    def connect_socket(self):
        if not self.socket_attempt_limit():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
                sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)
                self.socket = sock
                self.group  = (self.config.host, self.config.port)
                self.socket_connected = True
            except socekt.error:
                self.socket_connected = False
            
        return
    
    def socket_attempt_limit(self):
        '''
        limit rate of attempts to reopen UDP server
        '''
        if not hasattr(self, '_last_create_socket_attempt'):
            self._last_create_socket_attempt = time.time()
            return False
        else:
            curr_time = time.time()
            elapsed_time = curr_time - self._last_create_socket_attempt
            
            if elapsed_time > SOCKET_RETRY_INTERVAL:
                self._last_create_socket_attempt = curr_time
                return False
            return True
            
    
    def connect_vr(self):
        if not self.vr_attempt_limit():
            pass
        self.vr_connected = True #TODO
        
    def vr_attempt_limit(self):
        '''
        limit rate of attempts to reopen vr connection
        '''
        if not hasattr(self, '_last_create_vr_attempt'):
            self._last_create_vr_attempt = time.time()
            return False
        else:
            curr_time = time.time()
            elapsed_time = curr_time - self._last_create_vr_attempt
            
            if elapsed_time > SOCKET_RETRY_INTERVAL:
                self._last_create_vr_attempt = curr_time
                return False
            return True
    
    def read_vr(self):
        self.vr_state = OrientationQuaternion(qi = np.random.normal(),
                                              qj = np.random.normal(),
                                              qk = np.random.normal(),
                                              qr = np.random.normal())
        return True #TODO: false in event of error
    
    def write_multicast(self):
        try:
            self.socket.sendto(dill.dumps(self.vr_state), self.group)
        except socket.error:
            self.socket_connected = False
        return
  
  
if __name__ == '__main__':
    server = UDPServer(UDPConfig)
    for j in range(10):
        server.step()
             
