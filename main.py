from triad_openvr import *
import socket
from pydantic import BaseModel, Field
import json

from multiprocessing import Process, Pipe
import gui

CONNECT_RETRY_INTERVAL = 1
VR_UPDATE_INTERVAL = 1
MULTICAST_TTL = 2


class UDPConfig(BaseModel):
    host: str = Field(default='239.0.0.0')  # Administratively scoped IPv4 address space, only change last 3 numbers!
    port: int = Field(default=5007)         # port, any can be used.


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
    
    def unpack_vive_device(self, device, label):
        
        pose = device.get_pose_quaternion()
        vel = device.get_velocity()
        w = device.get_angular_velocity()
        
        if not pose and vel and w:
            self.valid = False
            return
        
        self.valid = True
        
        self.xi, self.xj, self.xk, self.qr, self.qi, self.qj, self.qk = pose
        self.v1, self.v2, self.v3 = vel
        self.w1, self.w2, self.w3 = w

        self.serial = device.get_serial()
        self.label = label

        # TODO: calibration
        
        return

    def to_bytes(self):
        json_data = json.dumps(self.json(), sort_keys=False)
        json_data = json_data + "\r"
        return json_data.encode()

    def from_bytes(self, json_msg):
        d = json.loads(json_msg.decode())
        self.parse_obj(d)
        return
        

class ViveServer:
    def __init__(self, config: UDPConfig = UDPConfig()):
        self.config = config
        
        self.vr = None
        self.socket = None
        self.group = None
        self.gui = None
        self.gui_pipe = None
        self.trackers = []
        self.tracker_labels = []
        self.references = []

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
            self.group  = (self.config.host, self.config.port)
        except socket.error:
            self.socket = None
            
        return
        
    def open_gui(self):
        if self.connect_attempt_limit('gui'):
            return
        self.gui_pipe, child_conn = Pipe()
        self.gui = gui.GuiManager(child_conn)
        self.gui_process = Process(target = self.gui.start)
        self.gui_process.start()
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
        # TODO: could put on timed event for better efficiency
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

    def update_gui(self):
        gui_msg = []

        for ref in self.references:
            ref_msg = TrackerState()
            ref_msg.unpack_vive_device(ref, 'ref')
            gui_msg.append(ref_msg)
        for device, device_label in zip(self.trackers, self.tracker_labels):
            msg = TrackerState()
            msg.unpack_vive_device(device, device_label)
            gui_msg.append(msg)

        self.gui_pipe.send(gui_msg)
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
        msgs = []
        for device, device_label in zip(self.trackers, self.tracker_labels):
            msg = TrackerState()
            msg.unpack_vive_device(device, device_label)
            msgs.append(msg.to_bytes())
        return msgs


if __name__ == '__main__':
    server = ViveServer()

    while True:
        server.step()

