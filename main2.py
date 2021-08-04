from triad_openvr import *
import socket
from pydantic import BaseModel, Field
import json
import numpy as np

from multiprocessing import Process, Pipe
import gui2

from scipy.spatial.transform import Rotation as rotation

CONNECT_RETRY_INTERVAL = 1
VR_UPDATE_INTERVAL = 1
MULTICAST_TTL = 2


class UDPConfig(BaseModel):
    host: str = Field(default='239.0.0.0')  # Administratively scoped IPv4 address space, only change last 3 numbers!
    port: int = Field(default=5007)         # port, any can be used.

class VRConfig(BaseModel):
    # Transform from world to vive
    xi: float = Field(default=0)
    xj: float = Field(default=0)
    xk: float = Field(default=0)
    qi: float = Field(default=0)
    qj: float = Field(default=0)
    qk: float = Field(default=0)
    qr: float = Field(default=1)

    name_mappings: dict = Field(default={})
    
class TrackerState(BaseModel):
    valid: int = Field(default=0)
    label: str = Field(default='')
    serial: str = Field(default='')
    charge: float = Field(default = 0)
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
    
    def unpack_vive_device(self, device, label = None):
        
        pose = device.get_pose_quaternion()
        vel = device.get_velocity()
        w = device.get_angular_velocity()
        
        if not pose and vel and w:
            self.valid = False
            return
        
        self.valid = True
        
        self.xi, self.xj, self.xk, self.qr, self.qi, self.qj, self.qk = pose
        
        # TODO: check conversion to body frame
        q = rotation.from_quat([self.qi, self.qj, self.qk, self.qr])
        self.v1, self.v2, self.v3 = q.apply(vel, inverse = True)
        self.w1, self.w2, self.w3 = q.apply(w,   inverse = True)
        
        self.charge = device.get_battery_percent()
        self.serial = device.get_serial()
        if label:
            self.label = label
        else:
            self.label = self.serial
        
        return
    
    def apply_calibration(self, origin, rotation_object):
        x = np.array([self.xi, self.xj, self.xk])
        x = x - origin
        self.xi, self.xj, self.xk = rotation_object.apply(x)
        
        q = rotation.from_quat([self.qi, self.qj, self.qk, self.qr])
        new_q = q * rotation_object
        self.qi, self.qj, self.qk, self.qr = new_q.as_quat()
        
            
    def to_bytes(self):
        json_data = json.dumps(self.json(), sort_keys=False)
        json_data = json_data + "\r"
        return json_data.encode()

    def from_bytes(self, json_msg):
        d = json.loads(json_msg.decode())
        self.parse_obj(d)
        return
        

class ViveServer:
    def __init__(self, udp_config: UDPConfig = UDPConfig(), vr_config: VRConfig = VRConfig()):
        self.udp_config = udp_config
        self.vr_config = vr_config
        
        self.should_close = False
        
        self.vr = None
        self.socket = None
        self.group = None
        self.gui = None
        self.gui_pipe = None
        self.trackers = []
        self.tracker_labels = []
        self.references = []
        
        self.calibration_origin = np.array([0,0,0])
        self.calibration_rotation = rotation.from_matrix(np.eye(3))
        self.reference_states = []
        self.tracker_states = []

    def step(self):
        #if not self.vr:
        #    self.open_vr()
        if not self.socket:
            self.open_socket()
        if not self.gui:
            self.open_gui()
        elif not self.gui.is_alive():
            self.should_close = True
        
        if True: #self.vr:
            #self.update_vr()
            
            self.poll_vr()
            self.apply_calibration()
            
            if self.gui:
                self.update_gui()
                
            if self.socket:
                self.broadcast()
        
        if self.should_close == True:
            return
        
                
    def open_vr(self):
        if self.connect_attempt_limit('vr'):
            return
        
        try:
            self.vr = TriadOpenVR()
        except:
            self.vr = None

    def open_socket(self):
        if self.connect_attempt_limit('socket'):
            return
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
            sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)
            self.socket = sock
            self.group  = (self.udp_config.host, self.udp_config.port)
        except socket.error:
            self.socket = None
            
        return
        
    def open_gui(self):
        if self.connect_attempt_limit('gui'):
            return
        self.gui_pipe, child_conn = Pipe()
        
        self.gui = Process(target = gui2.Window, args = (child_conn,))
        self.gui.start()
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
        return #TODO revert to hardware use
        if self.update_vr_limit():
            return

        self.vr.poll_vr_events()

        self.trackers = []
        self.tracker_labels = []
        self.references = []

        for device_label in self.vr.devices:
            device = self.vr.devices[device_label]
            if device.device_class == 'Tracker':
                self.trackers.append(device)
                self.tracker_labels.append(device_label)
            elif device.device_class == 'Tracking Reference':
                self.references.append(device)

        return
    
    def update_vr_limit(self):
        if not hasattr(self, '_last_vr_update'):
            self._last_vr_update = time.time()
            return False
        else:
            if time.time() - self._last_vr_update > VR_UPDATE_INTERVAL:
                self._last_vr_update = time.time()
                return False
            return True
    
    def poll_vr(self):
        '''self.reference_states = []
        self.tracker_states = []
        
        for ref in self.references:
            ref_msg = TrackerState()
            ref_msg.unpack_vive_device(ref)
            self.reference_states.append(ref_msg)
            
        for device, device_label in zip(self.trackers, self.tracker_labels):
            msg = TrackerState()
            msg.unpack_vive_device(device, device_label)
            self.tracker_states.append(msg)'''
        
        ref1 = TrackerState(xi = -1, xj = 4, serial = 'LHB1', label = 'ref')
        ref2 = TrackerState(xi = -1, xj = -7, serial = 'LHB2', label = 'ref')
        ref3 = TrackerState(xi = 3, xj = 2, serial = 'LHB3', label = 'ref')
        ref4 = TrackerState(xi = 5, xj = 7, serial = 'LHB4', label = 'ref')
        
        
        t1 = TrackerState(xi = 0.8, xj = 0.9, v1 = 4, serial = 'LHR1', label = 'T_1', charge = 10)
        t2 = TrackerState(xi = -0.3, xj = 0.5, v2 = 3, serial = 'LHR2', label = 'T_2', charge = 20)
        t3 = TrackerState(xi = 1.3, xj = 0.5, v1 = 0.3, v2 = 0.9, serial = 'LHR3', label = 'T_3', charge = 40)
        t4 = TrackerState(xi = 1.9, xj = 2.5, serial = 'LHR4', label = 'T_4', charge = 99.9)
        
        self.reference_states = [ref1, ref2, ref3, ref4]
        self.tracker_states = [t1,t2,t3,t4]
        return
    
    def apply_calibration(self):
        for state in self.reference_states:
            state.apply_calibration(self.calibration_origin, self.calibration_rotation)
        for state in self.tracker_states:
            state.apply_calibration(self.calibration_origin, self.calibration_rotation)
    
    def reset_calibration(self):
        self.calibration_origin = np.array([0,0,0])
        self.calibration_rotation = rotation.from_matrix(np.eye(3))
        
    def update_calibration(self, origin_label, x_label, y_label):
        # TODO: get states by name and poll data for some time to average out noise.
        print('updating calibration')
        
        to = TrackerState(xi = 0.8, xj = 0.9, serial = 'LHR1', label = 'T_1', charge = 10)
        tx = TrackerState(xi = -0.3, xj = 0.5, serial = 'LHR2', label = 'T_2', charge = 20)
        ty = TrackerState(xi = 1.3, xj = 0.5, serial = 'LHR3', label = 'T_3', charge = 40)
        
        x_origin = np.array([to.xi, to.xj, to.xk])
        x_x      = np.array([tx.xi, tx.xj, tx.xk])
        x_y      = np.array([ty.xi, ty.xj, ty.xk])
        
        dx = x_x - x_origin
        dx = dx / np.linalg.norm(dx)
        
        dy = x_y - x_origin
        dy = dy - dx * (np.dot(dx,dy))
        dy = dy / np.linalg.norm(dy)
        
        dz = np.cross(dx,dy)
        
        A = np.array([dx,dy,dz])
        
        self.calibration_origin = x_origin
        self.calibration_rotation = rotation.from_matrix(A)
        
        return
    
    def update_gui(self):
        gui_msg = []
         
        gui_msg = [*self.reference_states, *self.tracker_states]
        
        try:
            self.gui_pipe.send(gui_msg)           
        except ConnectionResetError:
            self.should_close = True
            self.gui = None
            return
        except BrokenPipeError:
            self.should_close = True
            self.gui = None
            return
        
        
        while self.gui_pipe.poll():
            try:
                msg = self.gui_pipe.recv()
            except EOFError:
                self.should_close = True
                self.gui = None
                return
                
            if msg[0] == 'calibrate':
                self.update_calibration(msg[1], msg[2], msg[3])
        
        return

    def broadcast(self):
        msgs = self.get_broadcast_messages()
        try:
            for msg in msgs:
                self.socket.sendto(msg, self.group)
        except socket.error:
            self.socket = None
        return

    def get_broadcast_messages(self):
        msgs = [msg.to_bytes() for msg in self.tracker_states]
        return msgs
    
    
    
    
    

if __name__ == '__main__':
    server = ViveServer()

    while not server.should_close:
        server.step()

