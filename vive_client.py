import socket
import struct

from dataclasses import dataclass, field
from models import TrackerState

@dataclass
class ViveConfig():
    address: str = field(default = '239.0.0.0')
    port: int = field(default = 5007)
    label: str = field(default = 'T_1')
    
    listen_until_msg_received:bool = field(default = True)
    
    

class ViveClient():
    def __init__(self, config: ViveConfig = None):
        if config is None: 
            config = ViveConfig()
            
        self.config = config
        self.sock = None
        self.state = TrackerState()
        self.step()
    
    
    def step(self):
        if not self.sock:
            self.connect()
        
        if self.sock:
            return self.listen()
        
        return
    
    def connect(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                # not available on Windows
                pass
            sock.bind((self.config.address, self.config.port))


            mreq = struct.pack('4sl', socket.inet_aton(self.config.address), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq) 
            
            self.sock = sock
        except socket.error:
            self.sock = None
    
    def listen(self):
        try:
            while True:
                new_state = self.state.from_bytes(self.sock.recv(1024))
                if new_state.label == self.config.label:
                    self.state = new_state
                    return self.state
                    
                if not self.config.listen_until_msg_received:
                    break
                    
        except socket.error:
            self.sock = None
            return -1
        return None
    
if __name__ == '__main__':
    cli = ViveClient()
    for j in range(50):
        print(cli.step())
            
            
            
            
            
