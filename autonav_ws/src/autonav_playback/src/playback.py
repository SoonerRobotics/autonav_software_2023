#!/usr/bin/env python3

from autonav_msgs.msg import IMUData, GPSFeedback, MotorFeedback, MotorInput, Position, MotorControllerDebug, ObjectDetection
from scr_core.state import DeviceStateEnum, SystemStateEnum, SystemMode
from sensor_msgs.msg import CompressedImage
from scr_msgs.msg import SystemState
from scr_core.node import Node, hash
from datetime import datetime
import cv_bridge
import shutil
import rclpy
import cv2
import os


CONFIG_RECORD_IMU = 0
CONFIG_RECORD_GPS = 1
CONFIG_RECORD_POSITION = 2
CONFIG_RECORD_FEEDBACK = 3
CONFIG_RECORD_OBJECTDETECTION = 4
CONFIG_RECORD_CAMERA = 5
CONFIG_RECORD_THRESHOLDED = 6
CONFIG_RECORD_EXPANDIFIED = 7
CONFIG_RECORD_MANUAL = 8
CONFIG_RECORD_AUTONOMOUS = 9
CONFIG_RECORD_INPUT = 10
CONFIG_RECORD_DEBUGFEEDBACK = 11


class PlaybackNode(Node):
    def __init__(self):
        super().__init__("autonav_playback")

    def configure(self):
        self.config.setBool(CONFIG_RECORD_IMU, True)
        self.config.setBool(CONFIG_RECORD_GPS, True)
        self.config.setBool(CONFIG_RECORD_POSITION, True)
        self.config.setBool(CONFIG_RECORD_FEEDBACK, True)
        self.config.setBool(CONFIG_RECORD_OBJECTDETECTION, True)
        self.config.setBool(CONFIG_RECORD_CAMERA, True)
        self.config.setBool(CONFIG_RECORD_THRESHOLDED, True)
        self.config.setBool(CONFIG_RECORD_EXPANDIFIED, True)
        self.config.setBool(CONFIG_RECORD_MANUAL, False)
        self.config.setBool(CONFIG_RECORD_AUTONOMOUS, True)
        self.config.setBool(CONFIG_RECORD_INPUT, True)
        self.config.setBool(CONFIG_RECORD_DEBUGFEEDBACK, True)
        
        self.file = None
        self.fileName = None
        self.bridge = cv_bridge.CvBridge()
        self.HOME_DIR = os.path.expanduser("~")
        
        self.imuSubscriber = self.create_subscription(IMUData, "/autonav/imu", self.imuCallback, 20)
        self.gpsSubscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.gpsCallback, 20)
        self.feedbackSubscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.feedbackCallback, 20)
        self.inputSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.inputCallback, 20)
        self.positionSubscriber = self.create_subscription(Position, "/autonav/position", self.positionCallback, 20)
        self.objectDetectionSubscriber = self.create_subscription(ObjectDetection, "/autonav/ObjectDetection", self.objectDetectionCallback, 20)
        self.motorControllerDebugSubscriber = self.create_subscription(MotorControllerDebug, "/autonav/MotorControllerDebug", self.motorControllerDebugCallback, 20)
        
        self.thresholdedSpaceSubscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image", self.thresholdedImageCallback, 20)
        self.expandifiedSpaceSubscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/expanded/image", self.expandifiedImageCallback, 20)
        self.cameraSubscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed", self.cameraImageCallback, 20)
        
        self.thresholdedIndex = 0
        self.expandifiedIndex = 0
        self.cameraIndex = 0
        self.startTime = datetime.now().timestamp()
        self.framerate = 2 if self.getSystemState().mode == SystemMode.SIMULATION else 15
        
        self.setDeviceState(DeviceStateEnum.OPERATING)
        
    def getTimestamp(self):
        return datetime.now().timestamp() - self.startTime
    
    def combineImagesIntoVideo(self, folder, name):
        IMAGES_PATH = os.path.join(self.HOME_DIR, ".scr", "playback", self.fileName, "images", folder)
        SAVE_PATH = os.path.join(self.HOME_DIR, ".scr", "playback", self.fileName)
        os.system(f"ffmpeg -r {self.framerate} -i {IMAGES_PATH}/%d.jpg -vcodec mpeg4 -y {SAVE_PATH}/{name}.mp4")
        
    def createFileName(self):
        time = datetime.now()
        timeFrmt = time.strftime("%Y-%m-%d_%H-%M-%S")
        stateFrmt = "autonomous" if self.getSystemState().state == SystemStateEnum.AUTONOMOUS else "manual"
        return f"{stateFrmt}_{timeFrmt}"
    
    def createLogEntry(self):
        self.fileName = self.createFileName()
        self.startTime = datetime.now().timestamp()
        
        # Create a folder at $HOME/.scr/playback/{fileName}
        # Create folders at $HOME/.scr/playback/{fileName}/images/{thresholded, expandified, camera}
        # Create a file at $HOME/.scr/playback/{fileName}/log.csv
        BASE_PATH = os.path.join(self.HOME_DIR, ".scr", "playback", self.fileName)
        os.makedirs(BASE_PATH, exist_ok = True)
        os.makedirs(os.path.join(BASE_PATH, "images", "thresholded"), exist_ok = True)
        os.makedirs(os.path.join(BASE_PATH, "images", "expandified"), exist_ok = True)
        os.makedirs(os.path.join(BASE_PATH, "images", "camera"), exist_ok = True)

        self.file = open(os.path.join(BASE_PATH, "log.csv"), "w")
        self.file.write("timestamp, type\n")
        
        self.thresholdedIndex = 0
        self.expandifiedIndex = 0
        self.cameraIndex = 0
        
        self.log(f"Recording playback data at {BASE_PATH}")

    def closeLogEntry(self):
        if self.file is None:
            return
        
        self.file.close()
        self.file = None
        
        # Zip up the folder at $HOME/.scr/playback/{fileName} and then delete it
        BASE_PATH = os.path.join(self.HOME_DIR, ".scr", "playback", self.fileName)
        self.combineImagesIntoVideo("thresholded", "thresholded")
        self.combineImagesIntoVideo("expandified", "expandified")
        self.combineImagesIntoVideo("camera", "camera")

        # Delete the images folder
        shutil.rmtree(os.path.join(BASE_PATH, "images"), ignore_errors = True)
        
        shutil.make_archive(BASE_PATH, "zip", BASE_PATH)
        SIZE_OF_ZIP = os.path.getsize(BASE_PATH + ".zip") / 1024 / 1024
        TIME_ELAPSED = datetime.now().timestamp() - self.startTime
        self.log(f"Saved playback data to {self.fileName}.zip ({SIZE_OF_ZIP:.3f} MB) over {TIME_ELAPSED:.3f} seconds")
        shutil.rmtree(BASE_PATH, ignore_errors = True)
        
    def writeToFile(self, msg: str):
        if self.file is None:
            return

        self.file.write(msg + "\n")
        
    def saveImageToDisk(self, img: CompressedImage, relative_path: str):
        if self.fileName is None:
            return

        IMAGE_PATH = os.path.join(self.HOME_DIR, ".scr", "playback", self.fileName, "images", relative_path)
        cv2Image = self.bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        cv2.imwrite(os.path.join(IMAGE_PATH, f"{self.thresholdedIndex}.jpg"), cv2Image)

    def transition(self, old: SystemState, updated: SystemState):            
        if old.state == SystemStateEnum.AUTONOMOUS and updated.state != SystemStateEnum.AUTONOMOUS:
            self.closeLogEntry()
            
        if old.state == SystemStateEnum.MANUAL and updated.state != SystemStateEnum.MANUAL:
            self.closeLogEntry()
           
        if old.state != SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.AUTONOMOUS and self.config.getBool(CONFIG_RECORD_AUTONOMOUS):
            self.createLogEntry()
        
        if old.state != SystemStateEnum.MANUAL and updated.state == SystemStateEnum.MANUAL and self.config.getBool(CONFIG_RECORD_MANUAL):
            self.createLogEntry()
            
    def imuCallback(self, msg: IMUData):
        if not self.config.getBool(CONFIG_RECORD_IMU):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_IMU, {msg.accel_x}, {msg.accel_y}, {msg.accel_z}, {msg.angular_x}, {msg.angular_y}, {msg.angular_z}, {msg.roll}, {msg.pitch}, {msg.yaw}")
            
    def gpsCallback(self, msg: GPSFeedback):
        if not self.config.getBool(CONFIG_RECORD_GPS):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_GPS, {msg.latitude}, {msg.longitude}, {msg.altitude}, {msg.gps_fix}, {msg.is_locked}, {msg.satellites}")
        
    def feedbackCallback(self, msg: MotorFeedback):
        if not self.config.getBool(CONFIG_RECORD_FEEDBACK):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_FEEDBACK, {msg.delta_x}, {msg.delta_y}, {msg.delta_theta}")
        
    def inputCallback(self, msg: MotorInput):
        if not self.config.getBool(CONFIG_RECORD_INPUT):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_INPUT, {msg.forward_velocity}, {msg.angular_velocity}")
        
    def positionCallback(self, msg: Position):
        if not self.config.getBool(CONFIG_RECORD_POSITION):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_POSITION, {msg.x}, {msg.y}, {msg.theta}, {msg.latitude}, {msg.longitude}")
        
    def objectDetectionCallback(self, msg: ObjectDetection):
        if not self.config.getBool(CONFIG_RECORD_OBJECTDETECTION):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_OBJECTDETECTION, {msg.sensor_1}, {msg.sensor_2}, {msg.sensor_3}")
        
    def motorControllerDebugCallback(self, msg: MotorControllerDebug):
        if not self.config.getBool(CONFIG_RECORD_DEBUGFEEDBACK):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_MOTORDEBUG, {msg.current_forward_velocity}, {msg.forward_velocity_setpoint}, {msg.current_angular_velocity}, {msg.angular_velocity_setpoint}, {msg.left_motor_output}, {msg.right_motor_output}")
            
    def thresholdedImageCallback(self, msg: CompressedImage):
        if not self.config.getBool(CONFIG_RECORD_THRESHOLDED):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_THRESHOLDED, /images/expandified/{self.thresholdedIndex}.jpg")
        self.saveImageToDisk(msg, "thresholded")
        self.thresholdedIndex += 1
        
        
    def expandifiedImageCallback(self, msg: CompressedImage):
        if not self.config.getBool(CONFIG_RECORD_EXPANDIFIED):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_EXPANDIFIED_IMAGE, /images/expandified/{self.expandifiedIndex}.jpg")
        self.saveImageToDisk(msg, "expandified")
        self.expandifiedIndex += 1
        
    def cameraImageCallback(self, msg: CompressedImage):
        if not self.config.getBool(CONFIG_RECORD_CAMERA):
            return
        
        self.writeToFile(f"{self.getTimestamp()}, ENTRY_CAMERA_IMAGE, /images/camera/{self.cameraIndex}.jpg")
        self.saveImageToDisk(msg, "camera")
        self.cameraIndex += 1
        


def main():
    rclpy.init()
    rclpy.spin(PlaybackNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
