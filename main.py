from triad_openvr import *

from pydantic import BaseModel, Field
import json

CONNECT_RETRY_INTERVAL = 1

class TrackerState(BaseModel):
    valid: int = Field(default=0)
    label: str = Field(default='')
    serial: str = Field(default='')
    xi: float = Field(default=0)
    xj: float = Field(default=0)
    xk: float = Field(default=0)
    qi: float = Field(default=0)
    qj: float = Field(default=0)
    qk: float = Field(default=0)
    qr: float = Field(default=1)
    v1: float = Field(default=0)
    v2: float = Field(default=0)
    v3: float = Field(default=0)
    w1: float = Field(default=0)
    w2: float = Field(default=0)
    w3: float = Field(default=0)
    
    def unpack_vive_device(self, device):
        
        pose = device.get_pose_quaternion()
        vel  = device.get_velocity()
        angvel = device.get_angular_velocity()
        
        if not pose and vel and angvel:
            self.valid = False
            return
        
        self.valid = True
        
        self.xi, self.xj, self.xk, self.qr, self.qi, self.qj, self.qk = pose
        self.v1, self.v2, self.v3 = vel
        self.w1, self.w2, self.w3 = angvel
        
        #TODO: label
        #TODO: serial
        #TODO: calibration
        
        return
        

class UDPConfig(BaseModel):
    host: str = Field(default = '239.0.0.0') # Administratively scoped IPv4 address space, only change last 3 numbers!
    port: int = Field(default = 5007)        # port, any can be used.
    
    
class ViveServer(config: UDPConfig = UDPConfig()):
    def __init__(self):
        self.config = config
        
        self.vr = None
        self.socket = None
        self.group = None
        self.gui = None
        
        self.step()

    def step(self):
        if not self.vr:
            self.open_vr()
        if not self.socket:
            self.open_socket()
        if not self.gui:
            self.open_gui()
            
        if self.vr:
            self.update_vr()
            
            if self.gui:
                self.update_gui()
                
            if self.socket:
                self.broadcast()
                
    def open_vr(self):
        if self.connect_attempt_limit('vr'): return
        
        try:
            self.vr = TriadOpenVR()
        except:
            self.vr = None

    def open_socket(self):
        if self.connect_attempt_limit('socket'): return
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
            sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)
            self.socket = sock
            self.group  = (self.config.host, self.config.port)
        except socekt.error:
            self.socket = None
            
        return
        
    def open_gui(self):
        if self.connect_attempt_limit('gui'): return
        #TODO
        return 
    
    def connect_attempt_limit(self, label):
        label_str = '_last_attempt_' + label
        if not hasattr(self, label_str):
            self.__setattr__(label_str, time.time())
            return False
        else:
            current_time = time.time()
            elapsed_time = current_time - self.__getattribute__(label_str)
            
            if elapsed_time > CONNECT_RETRY_INTERVAL:
                self.__setattr__(label_str, current_time)
                return False
            return True
    
       
    def update_vr(self):
        self.vr.poll_vr_events()
        #TODO sort items, update tracker list, update stuff to be send to GUI.
        return
    
    def update_gui(self):
        #TODO
        return

    def broadcast(self):
        msg = #TODO
        try:
            self.soscket.sendto(msg, self.group):
        except socket.error:
            self.socket = None
        return

    def get_broadcast_message(self):
        msgs = []
        for device_label in self.vr.devices:
            device = self.vr.devices[device_label]
            if device.device_class == 'Tracker':

                state = self.get_device_state(device, device_label)
                msgs.append(state)
        return msgs

    def get_device_state(self, device, label):
        xi, xj, xk, qr, qi, qj, qk = device.get_pose_quaternion()
        vi, vj, vk = device.get_velocity()
        wi, wj, wk = device.get_angular_velocity()

        return TrackerState(label = label,
                            xi = xi,
                            xj = xj,
                            xk = xk,
                            qi = qi,
                            qj = qj,
                            qk = qk,
                            qr = qr,
                            v1 = vi,
                            v2 = vj,
                            v3 = vk,
                            w1 = wi,
                            w2 = wj,
                            w3 = wk)


if __name__ == '__main__':
    server = ViveServer()

    for _ in range(1):
        server.step()
        msg = server.get_broadcast_message()
        dill_msg = dill.dumps(msg)
        import pdb
        pdb.set_trace()
        print(len(dill_msg))
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
