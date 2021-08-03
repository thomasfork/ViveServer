from abc import ABC, abstractmethod
import queue
from pathlib import Path
import math
import numpy as np

from multiprocessing import Pipe

from dearpygui.simple import *
from dearpygui.core import *


RED = [255, 0, 0, 255]
PURPLE = [128, 0, 128, 255]
GREEN = [0, 255, 0, 255]
BLUE = [0, 0, 255, 255]
GREY = [128, 128, 128, 255]
BLACK = [0, 0, 0, 255]

GRID_COLOR = [128, 128, 128, 50]
TRACKER_COLOR = [0, 255, 255, 255]
REFERENCE_COLOR = [255, 0, 255, 255]
CONTROLLER_COLOR = [255, 255, 255, 255]


class Scene:
    def __init__(self, width=1000, height=750, name="scene"):
        self.name = name
        self.width = width
        self.height = height
        self.is_open = False

        self.bounds = [0, 0, 0, 0]  # x left, y bot, x right, y top

    def add(self):
        add_spacing()
        add_drawing(self.name, width=self.width, height=self.height)
        self.is_open = True

    def update_references(self, references):
        xy = []
        for reference in references:
            xy.append([reference.xi, reference.xj])
        xy = np.array(xy)

        self.bounds[0], self.bounds[1] = np.min(xy, axis=0) - 0.5
        self.bounds[2], self.bounds[3] = np.max(xy, axis=0) + 0.5

        if self.is_open:
            for reference in references:
                self.draw_reference(reference)

    def update_trackers(self, trackers):
        if self.is_open:
            for tracker in trackers:
                self.draw_tracker(tracker)

    def xy2pixels(self, point):
        return [(point[0] - self.bounds[0]) / (self.bounds[2] - self.bounds[0]) * self.width,
                (point[1] - self.bounds[1]) / (self.bounds[3] - self.bounds[1]) * self.height]

    def draw_reference(self, reference):
        center = self.xy2pixels([reference.xi, reference.xj])
        diameter = 15
        label = reference.label

        draw_text(self.name, [center[0] + diameter + 5, center[1] + 6], color=REFERENCE_COLOR, text=label, size=13,
                  tag=f"{reference.serial}txt")
        draw_circle(self.name, center, diameter, REFERENCE_COLOR, fill=REFERENCE_COLOR, tag=f"{reference.serial}circle")

    def draw_tracker(self, tracker):
        center = self.xy2pixels([tracker.xi, tracker.xj])
        diameter = 10
        label = tracker.label
        yaw = 2 * np.arctan2(tracker.qk, tracker.qr)

        draw_text(self.name, [center[0] + diameter + 5, center[1] + 6], color=TRACKER_COLOR, text=label, size=13,
                  tag='abcdef')
        draw_circle(self.name, center, diameter, TRACKER_COLOR, fill=TRACKER_COLOR)#, tag=f"{tracker.serial}circle")

        radius = 20 + diameter/2
        yaw_line_pt = [center[0] + radius * np.cos(yaw), center[1] + radius * np.sin(yaw)]
        draw_line(self.name, center, yaw_line_pt, PURPLE, 3)#, tag=f"{tracker.serial}line")

    def draw_scales(self):
        tick_h = 5
        for x in range(0, self.width, 50):
            draw_line(self.name, [x, self.height], [x, 0], GRIDLINES, 1, tag=f"{x}xgridline")
            draw_line(self.name, [x, self.height], [x, self.height - tick_h], GREY, 1, tag=f"{x}xtick")
            x_real = self.real_pose_from_pixels([x, 0])[0]
            draw_text(self.name, [x, self.height - tick_h - 20], f'{round(x_real, 1)}m', color=GREY, size=13,
                      tag=f"{x}xticktext")
        for y in range(0, self.height, 50):
            draw_line(self.name, [0, y], [self.width, y], GRIDLINES, 1, tag=f"{y}ygridline")
            draw_line(self.name, [0, y], [tick_h, y], GREY, 1, tag=f"{y}ytick")
            y_real = self.real_pose_from_pixels([0, y])[1]
            draw_text(self.name, [tick_h + 5, y - 2], f'{round(y_real, 1)}m', color=GREY, size=13,
                      tag=f"{y}yticktext")

    def add_axes(self):
        length = 40
        draw_line("scene", self.center, [self.center[0], self.center[1] + length], GREEN, 3, tag="axis1")
        draw_line("scene", self.center, [self.center[0] + length, self.center[1]], RED, 3, tag="axis2")
        draw_circle("scene", self.center, 4, BLUE, fill=BLUE,
                    tag="axis3")

    def draw(self, device_state):
        clear_drawing("scene")
        draw_rectangle("scene", [0, 0], [self.width, self.height], BLACK, fill=BLACK, tag="backround")
        #self.draw_scales()
        #self.add_axes()
        for device in device_state:
            if 'tracker' in device:
                if device_state[device] is not None:
                    self.draw_tracker(device_state[device])

class GuiWindow:
    def __init__(self, gui_manager):
        self.gui_manager = gui_manager
        self.scene = Scene()
        # self.devices_page = DevicesPage(name="Devices List", gui_manager=self.gui_manager)
        # self.configuration_page = ConfigurationPage(name="Configuration", gui_manager=self.gui_manager)
        # self.calibrattion_page = CalibrationPage(name="Calibration", gui_manager=self.gui_manager)

    def show(self):
        ''' add_button("Save Configuration", callback=self.save_config)
        add_same_line()
        add_button("Refresh", callback=self.refresh)
        add_same_line()
        add_button("Calibrate", callback=self.calibrate)
        add_same_line()
        add_button("Test Calibration", callback=self.test_calibration)
        add_same_line()
        add_button("List Devices", callback=self.list_devices)
        add_same_line()
        add_button("Show Configuration", callback=self.show_configuration)
        add_same_line()
        add_button("Logs", callback=self.logs) '''
        self.scene.add()

    '''def save_config(self, sender, data):
        self.gui_manager.save_config()

    def refresh(self, sender, data):
        self.gui_manager.refresh_system()

    def calibrate(self, sender, data):
        self.calibrattion_page.show()

    def test_calibration(self, sender, data):
        pass

    def list_devices(self, sender, data):
        self.devices_page.show()

    def show_configuration(self, sender, data):
        self.configuration_page.show()

    def logs(self, sender, data):
        show_logger()'''

    def update(self, system_state: dict):
        self.scene.draw(system_state)
        if does_item_exist("Devices List"):
            self.devices_page.update(system_state)
        if does_item_exist("Configuration"):
            self.configuration_page.update(system_state)
        if does_item_exist("Calibration"):
            self.calibrattion_page.update(system_state)

    def clear(self):
        pass


class GuiManager:
    def __init__(self, pipe: Pipe):
        self.pipe = pipe
        self.window = GuiWindow(self)

    def run(self):
        return

    def step(self):
        while self.pipe.poll():
            messages = self.pipe.recv()

            trackers = []
            references = []

            for message in messages:
                if 'LHB' in message.serial:
                    references.append(message)
                elif 'LHR' in message.serial:
                    trackers.append(message)

            self.window.scene.update_references(references)
            self.window.scene.update_trackers(trackers)
        return

        # Will Run the main gui
    def start(self):
        with window("Vive Server", autosize=True, x_pos=20, y_pos=20):
            self.window.show()

        set_render_callback(self.step)
        start_dearpygui()