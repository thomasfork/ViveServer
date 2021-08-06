from pydantic import BaseModel, Field
import numpy as np
from scipy.spatial.transform import Rotation
import openvr


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

    def unpack_vr_device(self, vr_obj, pose_obj, index, vr_config: VRConfig = None):
        connected = pose_obj.bDeviceIsConnected
        valid = pose_obj.bPoseIsValid

        if not (connected and valid):
            self.valid = False
            return

        self.valid = True
        if not vr_config:
            vr_config = VRConfig()

        self.serial = vr_obj.getStringTrackedDeviceProperty(index, openvr.Prop_SerialNumber_String)
        if self.serial in vr_config.name_mappings.keys():
            self.label = vr_config.name_mappings[self.serial]
        else:
            self.label = self.serial

        try:
            self.charge = vr_obj.getFloatTrackedDeviceProperty(index, openvr.Prop_DeviceBatteryPercentage_Float)
        except openvr.error_code.TrackedProp_UnknownProperty:
            pass

        pose = pose_obj.mDeviceToAbsoluteTracking
        vel = pose_obj.vVelocity
        w = pose_obj.vAngularVelocity

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

        return

    def to_bytes(self):
        return self.json().encode()

    def from_bytes(self, json_msg):
        try:
            return self.parse_raw(json_msg.decode())
        except json.decoder.JSONDecodeError:
            return None
        

