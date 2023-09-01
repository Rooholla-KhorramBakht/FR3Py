import threading
import pyrealsense2 as rs
import numpy as np

class RealSenseCamera:
    def __init__(self, callback_fn = None, 
                       camera_serial_no=None, 
                       VGA = False,
                       color_fps=60,
                       depth_fps=90, 
                       enable_imu=True,
                       enable_depth=True, 
                       enable_color=True, 
                       enable_ir=False, 
                       emitter_enabled=True):
        
        self.callback_fn = callback_fn
        self.camera_serial_no = camera_serial_no
        self.VGA = VGA
        self.color_fps = color_fps
        self.depth_fps = depth_fps
        self.enable_depth = enable_depth
        self.enable_color = enable_color
        self.enable_ir = enable_ir
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.enable_imu = enable_imu
        if self.camera_serial_no is None:
            # Get device product line for setting a supporting resolution
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = self.config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            serial_no = str(device.get_info(rs.camera_info.serial_number))
            self.camera_serial_no = serial_no
        
        # Enable the streams for the connected device with the requested serial number
        print('Enabling streams for camera: ', self.camera_serial_no)
        self.config.enable_device(self.camera_serial_no)
        if VGA:
            img_size = (640, 480)
            if color_fps > 60:
                self.color_fps = 60
                print('Warning: VGA color fps cannot be higher than 60')
            if depth_fps > 90:
                self.depth_fps = 90
                print('Warning: VGA depth/infrared fps cannot be higher than 90')
        else:
            img_size = (1280, 720)
            if color_fps > 30:
                self.color_fps = 30
                print('Warning: HD color fps cannot be higher than 30')
            if depth_fps > 30:
                self.depth_fps = 30
                print('Warning: HD depth/infrared fps cannot be higher than 30')
        
        if enable_depth:
            self.config.enable_stream(rs.stream.depth, img_size[0], img_size[1], rs.format.z16, self.depth_fps)
        if enable_color:
            self.config.enable_stream(rs.stream.color, img_size[0], img_size[1], rs.format.bgr8, self.color_fps)
        if enable_ir:
            self.config.enable_stream(rs.stream.infrared, 1, img_size[0], img_size[1], rs.format.y8, self.depth_fps)
            self.config.enable_stream(rs.stream.infrared, 2, img_size[0], img_size[1], rs.format.y8, self.depth_fps)
        if self.enable_imu:
            self.config.enable_stream(rs.stream.accel)
            self.config.enable_stream(rs.stream.gyro)

        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        if emitter_enabled:
            self.depth_sensor.set_option(rs.option.emitter_enabled, 1)
        else:
            self.depth_sensor.set_option(rs.option.emitter_enabled, 0)
        
        # Start the thread for grabbing frames
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run_grab_frames)
        self._thread.start()

    def _run_grab_frames(self):
        while not self._stop_event.is_set():
            self.grab_frames()
            if self.callback_fn is not None:
                self.callback_fn(self.color_frame, self.depth_frame, self.ir1_frame, self.ir2_frame)

    def close(self):
        # Stop the thread
        self._stop_event.set()
        self._thread.join()
        # Stop the pipeline
        self.pipeline.stop()

    def grab_frames(self):
        frames = self.pipeline.wait_for_frames()
        if frames is None:
            print('Warning: failed to grab frames')
            self.close()
            
        if self.enable_depth:
            self.depth_frame = np.asanyarray(frames.get_depth_frame().get_data())
        else:
            self.depth_frame = None
        if self.enable_color:
            self.color_frame = np.asanyarray(frames.get_color_frame().get_data())
        else:
            self.color_frame = None
        if self.enable_ir:
            self.ir1_frame =   np.asanyarray(frames.get_infrared_frame(1).get_data())
            self.ir2_frame =   np.asanyarray(frames.get_infrared_frame(2).get_data())
        else:
            self.ir1_frame = None
            self.ir2_frame = None
        
        if self.enable_imu:
            self.accel_frame = frames.first_or_default(rs.stream.accel)
            self.gyro_frame = frames.first_or_default(rs.stream.gyro)
            # Optionally convert the IMU frames to arrays as needed, e.g., np.asanyarray(...)
        else:
            self.accel_frame = None
            self.gyro_frame = None