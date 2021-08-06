import socket
import numpy as np
from multiprocessing import Process, Queue
import pathlib
import os
from scipy.spatial.transform import Rotation

import openvr
from triad_openvr import *

import gui
from models import * 


# TODO does not properly end in pycharm on window close, even though processes all end...

CONNECT_RETRY_INTERVAL = 1
VR_UPDATE_INTERVAL = 1
MULTICAST_TTL = 20


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
