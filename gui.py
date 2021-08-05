import numpy as np
import dearpygui.dearpygui as dpg

from pydantic import BaseModel, Field
import time

from multiprocessing import Process, Pipe


RED = [255,0,0,255]
PURPLE = [128, 0, 128, 255]
GREEN = [0, 255, 0, 255]
BLUE = [0, 0, 255, 255]
GREY = [128, 128, 128, 255]
BLACK = [0, 0, 0, 255]
CLEAR = [0,0,0,0]

GRID_COLOR = [128, 128, 128, 50]
GRID_TEXT_COLOR = [255,255,0,255]
TRACKER_COLOR = [0, 255, 255, 255]
REFERENCE_COLOR = [255, 0, 255, 255]
CONTROLLER_COLOR = [255, 255, 255, 255]

CLEAR_DELAY = 1
VELOCITY_TIME_MULT = 1


class GuiConfig(BaseModel):
    vr_origin: str = Field(default = 'T_3')
    vr_x : str = Field(default = 'T_4')
    vr_y : str = Field(default = 'T_1')
    

class Scene:
    def __init__(self):
        self.width = dpg.get_viewport_width()
        self.height = dpg.get_viewport_height()
        self.is_open = False

        self.bounds = np.asarray([-1, -1, 10, 5])  # x left, y bot, x right, y top
        self.resized = True
        
        self.canvas = dpg.add_viewport_drawlist(front = False)
        self.grid_canvas  = dpg.add_viewport_drawlist(front = False)
        
    def update(self, references, trackers):
        
        dpg.delete_item(self.canvas, children_only = True)
        if references:
            self.update_references(references)
        if trackers:
            self.update_trackers(trackers)
            
        if self.resized:
            dpg.delete_item(self.grid_canvas, children_only = True)
            self.draw_grid()
            self.resized = False
        
        return
        
    def update_references(self, references):
        xy = []
        for reference in references:
            xy.append([reference.xi, reference.xj])
        xy = np.array(xy)
        
        x_min, y_min = np.min(xy, axis=0) - 1
        x_max, y_max = np.max(xy, axis=0) + 1
        bounds = np.asarray([x_min, y_min, x_max, y_max])
        
        for reference in references:
            self.draw_reference(reference)
        
        if (bounds != self.bounds).any():
            self.bounds = bounds
            self.resized = True
            print('resizing')
        return False

    def update_trackers(self, trackers):
        for tracker in trackers:
            self.draw_tracker(tracker)

    def xy2pixels(self, point, differential = False):
        if differential:
            return [point[0] / (self.bounds[2] - self.bounds[0]) * self.width,
                    point[1] / (self.bounds[3] - self.bounds[1]) * self.height]
        else:
            return [(point[0] - self.bounds[0]) / (self.bounds[2] - self.bounds[0]) * self.width,
                 (1-(point[1] - self.bounds[1]) / (self.bounds[3] - self.bounds[1])) * self.height]

    def draw_reference(self, reference):
        center = self.xy2pixels([reference.xi, reference.xj])
        diameter = 15
        label = reference.label

        dpg.draw_text([center[0] + diameter + 5, center[1] + 6], color=REFERENCE_COLOR, text=label, size=18, parent = self.canvas)
        dpg.draw_circle(center, diameter, color=REFERENCE_COLOR, fill=REFERENCE_COLOR, parent = self.canvas)
        return

    def draw_tracker(self, tracker):
        center = self.xy2pixels([tracker.xi, tracker.xj])
        diameter = 10
        label = tracker.label
        
        yaw = 2 * np.arctan2(tracker.qk, tracker.qr)
        line_length = 20 + diameter/2
        yaw_line_pt = [center[0] + line_length * np.cos(yaw), center[1] - line_length * np.sin(yaw)]
        
        vel_delta = self.xy2pixels([tracker.v1 * np.cos(yaw) * VELOCITY_TIME_MULT - tracker.v2 * np.sin(yaw) * VELOCITY_TIME_MULT,
                                    -tracker.v1 * np.sin(yaw) * VELOCITY_TIME_MULT - tracker.v2 * np.cos(yaw) * VELOCITY_TIME_MULT], True)
        
        vel_pt = [center[0] + vel_delta[0], center[1] + vel_delta[1]]

        dpg.draw_text([center[0] + diameter + 5, center[1] + 6], color=TRACKER_COLOR, text=label, size=18, parent = self.canvas)
        dpg.draw_circle(center, diameter, color = TRACKER_COLOR, fill=TRACKER_COLOR, parent = self.canvas)
        dpg.draw_line(center, yaw_line_pt, color = PURPLE, thickness = 3, parent = self.canvas)
        dpg.draw_arrow(vel_pt, center, color = RED, thickness = 3, parent = self.canvas)
        
        return

    def draw_grid(self):
        grid_spacing = 0.5
        
        x_min = np.ceil(self.bounds[0]/grid_spacing) * grid_spacing
        x_max = np.floor(self.bounds[2]/grid_spacing) * grid_spacing
        y_min = np.ceil(self.bounds[1]/grid_spacing) * grid_spacing
        y_max = np.floor(self.bounds[3]/grid_spacing) * grid_spacing
        
        xc, yc = self.xy2pixels([0,0])
        
        for x in np.arange(x_min, x_max+0.1, grid_spacing):
            p = self.xy2pixels([x, 0])
            xp = p[0]
            
            if abs(x-0) < grid_spacing/2:
                line = dpg.draw_line([xp, 0], [xp,self.height], color = GRID_COLOR, thickness = 5, parent = self.grid_canvas)
            else:
                line = dpg.draw_line([xp, 0], [xp,self.height], color = GRID_COLOR, thickness = 1, parent = self.grid_canvas)
            text = dpg.draw_text([xp + 8,yc ], text = '%0.1f'%x, color = GRID_TEXT_COLOR, parent = self.grid_canvas, size = 18)
            
        for y in np.arange(y_min, y_max, grid_spacing):
            p = self.xy2pixels([0,y])
            yp = p[1]
            if abs(y-0) < grid_spacing/2:
                line = dpg.draw_line([0, yp], [self.width, yp], color = GRID_COLOR, thickness = 5, parent = self.grid_canvas)
            else:
                line = dpg.draw_line([0, yp], [self.width, yp], color = GRID_COLOR, thickness = 1, parent = self.grid_canvas)
            text = dpg.draw_text([xc + 8,yp ], text = '%0.1f'%y, color = GRID_TEXT_COLOR, parent = self.grid_canvas, size = 18)
        
        return


class PopupWindow():
    def __init__(self):
        self.window = dpg.add_window(pos = [40,40], width = 500, height = 500, on_close = self.on_close)
        self.hide()
        self.visible = False
    
    def clear(self):
        dpg.delete_item(self.window, children_only = True)
         
    def hide(self):  
        dpg.hide_item(self.window)
        self.visible = False
    
    def show(self):
        dpg.show_item(self.window)
        self.visible = True
    
    def on_close(self):
        self.visible = False
   
    def toggle_visibility(self):
        if self.visible:
            self.hide()
        else:
            self.show()
            
            
class DeviceWindow(PopupWindow):
    
    def update(self, references, trackers):
        self.clear()
        
        for reference in references:
            dpg.add_text(reference.label, parent = self.window)
            dpg.add_same_line(parent = self.window)
            dpg.add_text(reference.serial, parent = self.window)
            dpg.add_same_line(parent = self.window)
            dpg.add_text('x: %0.3f y:%0.3f z:%0.3f'%(reference.xi, reference.xj, reference.xk), parent = self.window)
            dpg.add_spacing(parent = self.window)
            pass
            
        dpg.add_spacing(count = 5, parent = self.window)
        
        for tracker in trackers:
            dpg.add_text(tracker.label, parent = self.window)
            dpg.add_same_line(parent = self.window)
            dpg.add_text(tracker.serial, parent = self.window)
            dpg.add_same_line(parent = self.window)
            dpg.add_text('x: %0.3f y:%0.3f z:%0.3f'%(tracker.xi, tracker.xj, tracker.xk), parent = self.window)
            dpg.add_slider_float(no_input = True, min_value = 0, max_value = 1, default_value = tracker.charge, enabled = False, parent = self.window)
            dpg.add_spacing(parent = self.window)
        return


class CalibrationWindow(PopupWindow):
    
    def __init__(self, calibration_function, config: GuiConfig = GuiConfig()):
        self.config = config
        self.calibration_function = calibration_function
        
        self.window = dpg.add_window(pos = [40,40], width = 500, height = 500, on_close = self.on_close)
        self.hide()
        labels = '[]'
        
        dpg.add_text('Please select a tracker for \n each axis. Available trackers \n are listed below for convenience:', parent = self.window)
        dpg.add_spacing(parent=self.window)
        self.labels = dpg.add_text(str(labels), parent = self.window)
        self.text_origin = dpg.add_input_text(label = 'origin', default_value=self.config.vr_origin, parent = self.window)
        self.text_x = dpg.add_input_text(label = 'x axis', default_value=self.config.vr_x, parent = self.window)
        self.text_y = dpg.add_input_text(label = 'y axis', default_value=self.config.vr_y, parent = self.window)

        dpg.add_button(label='Start calibration', callback=self.run_calibration, parent = self.window)
        
    def update(self, references, trackers):
        labels = str([tracker.label for tracker in trackers])
        dpg.set_value(self.labels, labels)
        
        return

    def run_calibration(self):
        vr_origin = dpg.get_value(self.text_origin)
        vr_x = dpg.get_value(self.text_x)
        vr_y = dpg.get_value(self.text_y)
        
        self.calibration_function(vr_origin, vr_x, vr_y)
        
        return
        
        
class ConfigWindow(PopupWindow):
    
    def update(self, config):
        self.clear()
        dpg.add_text('TODO', parent = self.window)
        
class Window:
    def __init__(self, pipe:Pipe = None, config: GuiConfig = GuiConfig()):
        self.pipe = pipe
        self.config = config
        self.vr_config = None #TODO get vr cooridnate config from server
        
        self.create_window()
        self.scene = Scene()
        self.device_window = DeviceWindow()
        self.calibration_window = CalibrationWindow(self.recalibrate, self.config)
        self.config_window = ConfigWindow()
        
        self.last_msg_time = time.time()
        self.run()
    
    def create_window(self):
        dpg.setup_viewport()
        dpg.set_viewport_width(800)
        dpg.set_viewport_height(800)
        
        bar = dpg.add_viewport_menu_bar() 
        dpg.add_button(label = "Save Configuration", callback=self.save_config, parent = bar)
        dpg.add_button(label = "Calibrate", callback=self.show_calibration_window, parent = bar)
        dpg.add_button(label = "List Devices", callback=self.list_devices, parent = bar)
        dpg.add_button(label = "Show Configuration", callback=self.show_config, parent = bar)
            
        dpg.set_viewport_resize_callback(self.resize_window)
        
    def save_config(self):
        return
    
    def show_config(self):
        self.config_window.toggle_visibility()
        return
        
    def show_calibration_window(self):
        self.calibration_window.toggle_visibility()
        return
    
    def recalibrate(self, vr_origin, vr_x, vr_y):
        msg = ['calibrate', vr_origin, vr_x, vr_y]
        self.pipe.send(msg)
        return
        
    def list_devices(self):
        self.device_window.toggle_visibility()
        
    def resize_window(self):
        self.scene.width = dpg.get_viewport_width()
        self.scene.height = dpg.get_viewport_height()
        self.scene.resized = True
        return
        
    def run(self):
        while dpg.is_dearpygui_running():
            self.step()
        dpg.cleanup_dearpygui()
        return
    
    def step(self):
        references = []
        trackers = []
        while self.pipe and self.pipe.poll():
            references = []
            trackers = []
            
            messages = self.pipe.recv()
            for message in messages:
                if 'LHB' in message.serial:
                    references.append(message)
                elif 'LHR' in message.serial:
                    trackers.append(message)

            
            self.scene.update(references, trackers)
            self.device_window.update(references, trackers)
            self.calibration_window.update(references, trackers)
            self.config_window.update(self.config)
            self.last_msg_time = time.time()
            
        if time.time() - self.last_msg_time > CLEAR_DELAY: # in the absence of data for too long, clear the screen.
            self.scene.update(references, trackers)
            self.device_window.update(references, trackers)
            self.calibration_window.update(references, trackers)
            self.config_window.update(self.config)
            
        dpg.render_dearpygui_frame()    
        return   


if __name__ == '__main__':
    gui_pipe, child_conn = Pipe()
    gui_process = Process(target = Window, args = (child_conn,))
    gui_process.start()
    gui_process.join()
