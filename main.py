import socket
from pydantic import BaseModel, Field
import numpy as np
from multiprocessing import Process, Queue
import pathlib
import os
from scipy.spatial.transform import Rotation

import openvr
from triad_openvr import *

import gui


# TODO does not properly end in pycharm on window close, even though processes all end...

CONNECT_RETRY_INTERVAL = 1
VR_UPDATE_INTERVAL = 1
MULTICAST_TTL = 20


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

    name_mappings: dict = Field(default={'LHR-0A321FAF': 'T_3',
                                         'LHR-207B3EA4': 'T_2',
                                         'LHR-55804C5D': 'T_1',
                                         'LHR-77E956FA': 'T_4',
                                         'LHR-F783E5C2': 'T_5'})


class TrackerState(BaseModel):
    valid: int = Field(default=0)
    label: str = Field(default='')
    serial: str = Field(default='')
    charge: float = Field(default=0)
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

    def e1(self):
        return np.array([[1 - 2 * self.qj ** 2 - 2 * self.qk ** 2,
                          2 * (self.qi * self.qj + self.qk * self.qr),
                          2 * (self.qi * self.qk - self.qj * self.qr)]]).T

    def e2(self):
        return np.array([[2 * (self.qi * self.qj - self.qk * self.qr),
                          1 - 2 * self.qi ** 2 - 2 * self.qk ** 2,
                          2 * (self.qj * self.qk + self.qi * self.qr)]]).T

    def e3(self):
        return np.array([[2 * (self.qi * self.qk + self.qj * self.qr),
                          2 * (self.qj * self.qk - self.qi * self.qr),
                          1 - 2 * self.qi ** 2 - 2 * self.qj ** 2]]).T

    def unpack_vive_device(self, device, label=None, vr_config: VRConfig = None):
        pose = device.get_pose_matrix()
        vel = device.get_velocity()
        w = device.get_angular_velocity()
        
        if not (pose and vel and w):
            self.valid = False
            return
        
        self.valid = True
        if not vr_config:
            vr_config = VRConfig()

        self.serial = device.get_serial()
        if self.serial in vr_config.name_mappings.keys():
            self.label = vr_config.name_mappings[self.serial]
        elif label:
            self.label = label
        else:
            self.label = self.serial

        origin = np.array([vr_config.xi, vr_config.xj, vr_config.xk])
        rotation = Rotation.from_quat([vr_config.qi, vr_config.qj, vr_config.qk, vr_config.qr])

        # pose in VR frame
        x = np.array([pose[0][3], pose[1][3], pose[2][3]])
        q = Rotation.from_matrix([[pose[0][0], pose[0][1], pose[0][2]],
                                  [pose[1][0], pose[1][1], pose[1][2]],
                                  [pose[2][0], pose[2][1], pose[2][2]]])
        # velocity in VR frame
        vi, vj, vk = vel
        wi, wj, wk = w

        # transform position to calibrated frame
        x = x - origin
        self.xi, self.xj, self.xk = rotation.apply(x)

        # transform orientation to calibrated frame
        q = rotation * q

        # transform velocity to calibrated frame
        vi, vj, vk = rotation.apply([vi, vj, vk])
        wi, wj, wk = rotation.apply([wi, wj, wk])

        # transform pose from HTC vive frame to vehicle frame, with charge port pointing forwards

        body_rot = Rotation.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        q = (q * body_rot)
        self.qi, self.qj, self.qk, self.qr = q.as_quat()

        # use body frame pose to convert calibrated frame velocity to body frame
        self.v1, self.v2, self.v3 = q.apply([vi, vj, vk], inverse=True)
        self.w1, self.w2, self.w3 = q.apply([wi, vj, wk], inverse=True)

        try:
            self.charge = device.get_battery_percent()
        except openvr.error_code.TrackedProp_UnknownProperty:
            pass
        return

    def to_bytes(self):
        return self.json().encode()

    def from_bytes(self, json_msg):
        return self.parse_raw(json_msg.decode())
        

class ViveServer:
    def __init__(self, udp_config: UDPConfig = UDPConfig()):
        self.udp_config = udp_config
        self.vr_config = VRConfig()
        
        self.should_close = False
        
        self.vr = None
        self._last_vr_update = time.time() - VR_UPDATE_INTERVAL
        self.socket = None
        self.group = None
        self.gui = None
        self.gui_in = None
        self.gui_out = None
        self._gui_config_updated = False
        self.trackers = []
        self.tracker_labels = []
        self.references = []

        self.reference_states = []
        self.tracker_states = []

        self.load_calibration()

    def step(self):
        if not self.vr:
            self.open_vr()
        if not self.socket:
            self.open_socket()
        if not self.gui:
            self.open_gui()
        elif not self.gui.is_alive():
            self.should_close = True

        if self.vr:
            self.update_vr()

        if self.vr:  # update can delete vr
            self.poll_vr()
            
            if self.gui:
                self.update_gui()
                
            if self.socket:
                self.broadcast()

        if self.should_close:
            return

    def close(self):
        if self.gui_in:
            self.gui_in.cancel_join_thread()
        if self.gui_out:
            self.gui_out.cancel_join_thread()

    def open_vr(self):
        if self.connect_attempt_limit('vr'):
            return
        
        try:
            self.vr = TriadOpenVR()
            print('connected vr')
        except openvr.error_code.InitError_Init_HmdNotFoundPresenceFailed:
            print('no HMD found')
            self.vr = None

    def open_socket(self):
        if self.connect_attempt_limit('socket'):
            return
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
            sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)
            self.socket = sock
            self.group = (self.udp_config.host, self.udp_config.port)
            print('connected socket')
        except socket.error:
            self.socket = None
            
        return
        
    def open_gui(self):
        if self.connect_attempt_limit('gui'):
            return
        self.gui_in, self.gui_out = Queue(), Queue()
        
        self.gui = Process(target=gui.Window, args=(self.gui_in, self.gui_out))
        self.gui.start()
        print('connected gui')
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
        if self.update_vr_limit():
            return

        self.trackers = []
        self.tracker_labels = []
        self.references = []

        try:
            self.vr.poll_vr_events()
        except KeyError:
            self.vr = None
            return

        if not self.vr.devices:
            self.vr = None
            return

        for device_label in self.vr.devices:
            device = self.vr.devices[device_label]
            if device.device_class == 'Tracker':
                self.trackers.append(device)
                self.tracker_labels.append(device_label)
            elif device.device_class == 'Tracking Reference':
                self.references.append(device)

        return
    
    def update_vr_limit(self):
        if time.time() - self._last_vr_update > VR_UPDATE_INTERVAL:
            self._last_vr_update = time.time()
            return False
        return True
    
    def poll_vr(self):
        self.reference_states = []
        self.tracker_states = []
        
        for ref in self.references:
            ref_msg = TrackerState()
            ref_msg.unpack_vive_device(ref, vr_config=self.vr_config)
            self.reference_states.append(ref_msg)
            
        for device, device_label in zip(self.trackers, self.tracker_labels):
            msg = TrackerState()
            msg.unpack_vive_device(device, device_label, vr_config=self.vr_config)
            self.tracker_states.append(msg)
        return
    
    def update_gui(self):
        gui_msg = [*self.reference_states, *self.tracker_states]

        try:
            self.gui_in.put(gui_msg)
        except ConnectionResetError:
            self.should_close = True
            self.gui = None
            return
        except BrokenPipeError:
            self.should_close = True
            self.gui = None
            return

        if not self._gui_config_updated:
            self.gui_in.put([self.vr_config])
            self._gui_config_updated = True

        while not self.gui_out.empty():
            msg = self.gui_out.get()
                
            if msg[0] == 'calibrate':
                self.update_calibration(msg[1], msg[2], msg[3])
            elif msg[0] == 'restart_vr':
                self.vr = None
            elif msg[0] == 'save_calibration':
                self.save_calibration()
                
        return

    def broadcast(self):
        msgs = self.get_broadcast_messages()

        try:
            for msg in msgs:
                self.socket.sendto(msg, self.group)
        except socket.error:
            self.socket = None
            print('broadcast error')
            return
        return

    def get_broadcast_messages(self):
        msgs = [msg.to_bytes() for msg in self.tracker_states]
        return msgs

    def update_calibration(self, origin_label, x_label, y_label):
        present_labels = [device.label for device in self.tracker_states]

        try:
            origin_index = present_labels.index(origin_label)
            x_index = present_labels.index(x_label)
            y_index = present_labels.index(y_label)
        except ValueError:
            print('calibration trackers not all present')
            return
        
        self.reset_calibration()
        self.poll_vr()

        print('updating calibration')

        to = self.tracker_states[origin_index]
        tx = self.tracker_states[x_index]
        ty = self.tracker_states[y_index]

        x_origin = np.array([to.xi, to.xj, to.xk])
        x_x = np.array([tx.xi, tx.xj, tx.xk])
        x_y = np.array([ty.xi, ty.xj, ty.xk])

        dx = x_x - x_origin
        dx = dx / np.linalg.norm(dx)

        dy = x_y - x_origin
        dy = dy - dx * (np.dot(dx, dy))
        dy = dy / np.linalg.norm(dy)

        dz = np.cross(dx, dy)

        basis_matrix = np.array([dx, dy, dz])
        rotation = Rotation.from_matrix(basis_matrix)

        # update config object as well
        self.vr_config.xi, self.vr_config.xj, self.vr_config.xk = x_origin
        self.vr_config.qi, self.vr_config.qj, self.vr_config.qk, self.vr_config.qr = rotation.as_quat()

        self._gui_config_updated = False
        return

    def save_calibration(self):
        filename = get_config_path()
        with open(filename, 'w+') as f:
            data = self.vr_config.json()
            f.seek(0)
            f.write(data)
            f.truncate()
        return

    def load_calibration(self):
        filename = get_config_path()
        if os.path.exists(filename):
            with open(filename) as f:
                msg = f.readline()
                self.vr_config = self.vr_config.parse_raw(msg)
            self._gui_config_updated = False
        return

    def reset_calibration(self):
        blank_config = VRConfig()
        blank_config.name_mappings = self.vr_config.name_mappings
        self.vr_config = blank_config
        self._gui_config_updated = False
        return


def get_config_path():
    path = pathlib.Path(__file__).parent.resolve()
    filename = path / 'config.txt'
    return filename


if __name__ == '__main__':
    server = ViveServer()

    while not server.should_close:
        server.step()
    server.close()
    print('server shutdown')
