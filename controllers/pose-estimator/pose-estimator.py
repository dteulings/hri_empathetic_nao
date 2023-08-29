"""tutorial1_tracker controller.

Student group:
Zuzanna Fendor s1008877
Steffen Ricklin s1009136

Code estimation code used from (08-Dec-2021):
https://www.analyticsvidhya.com/blog/2021/05/pose-estimation-using-opencv/
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard, Display, Motion, Camera, PositionSensor
import numpy as np
import math
import cv2
import os
import sys
import mediapipe as mp
import time
from PoseModule import PoseDetector


class MyRobot(Robot):
    
    def __init__(self, ext_camera_flag):
        super(MyRobot, self).__init__()
        print('> Starting robot controller')
        
        # self.timeStep = 32 # Milisecs to process the data (loop frequency) - Use int(self.getBasicTimeStep()) for default
        self.timeStep = int(self.getBasicTimeStep())
        self.state = 0 # Idle starts for selecting different states
        
        ## SENSORS
        self.HeadYawS = self.getDevice('HeadYawS')
        self.HeadPitchS = self.getDevice('HeadPitchS')
        self.HeadYawS.enable(self.timeStep)
        self.HeadPitchS.enable(self.timeStep)
        
        ## Execute one step to get the initial position
        self.step(self.timeStep) 
        
        ## CAMERAS
        #-external camera
        self.ext_camera = ext_camera_flag        
        self.displayCamExt = self.getDevice('CameraExt')
        if self.ext_camera:
            self.cameraExt = cv2.VideoCapture(0)
        #-get the face cascade for finding faces in the external camera
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        #-bottom camera of the robot
        self.camera_bot = self.getDevice('CameraBottom')
        self.camera_bot.enable(self.timeStep)
        # self.camera_bot.enable(10)
        
        ## ACTUATORS
        self.head_yaw = self.getDevice("HeadYaw")
        self.head_pitch = self.getDevice("HeadPitch")
        # self.head_yaw.setPosition(float('inf'))
        # self.head_pitch.setPosition(float('inf'))
        self.head_yaw.setVelocity(1.)
        self.head_pitch.setVelocity(1.)
        
       
        ## KEYBOARD
        self.keyboard.enable(self.timeStep)
        self.keyboard = self.getKeyboard()

    def camera_read_external(self):
        """Captures the external camera frames and
        returns the image downsampled by 2 
        """
        img = []
        if self.ext_camera:
            # Capture frame-by-frame
            ret, frame = self.cameraExt.read()
            # Our operations on the frame come here
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # From openCV BGR to RGB
            img = cv2.resize(img, None, fx=0.5, fy=0.5) # image downsampled by 2              
        return img
            
    def image_to_display(self, img):
        """Displays the image on the webots camera display interface
        """
        if self.ext_camera:
            height, width, channels = img.shape
            imageRef = self.displayCamExt.imageNew(cv2.transpose(img).tolist(), Display.RGB, width, height)
            self.displayCamExt.imagePaste(imageRef, 0, 0)
    
    def print_gps(self):
        gps_data = self.gps.getValues();
        print('----------gps----------')
        print(' [x y z] =  [' + str(gps_data[0]) + ',' + str(gps_data[1]) + ',' + str(gps_data[2]) + ']' )
        
    def printHelp(self):
        print(
            'Commands:\n'
            ' H for displaying the commands\n'
            ' G for print the gps\n'
        )
            
    ############################
    #### RUN FACE FOLLOWER #####    
    ############################
    def run_face_follower(self):
        """
        Face following main function
        
        Main control loop: perform simulation steps of self.timeStep milliseconds
        and leave the loop when the simulation is over
        """        
        while self.step(self.timeStep) != -1:
            img = self.camera_read_external()
            
            # Write your controller here
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # use detectMultiScale from opencv using Haar descriptors
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            for face in faces:  # actually the robot just looks at the last detected face or skips the for-loop if faces is empty
                face_center = self.compute_face_center(face, img)
                self.look_at(face_center, velocity=1.)
            self.image_to_display(img)
            
        # finallize class. Destroy external camera.
        if self.ext_camera:
            self.cameraExt.release()
    
    def run_image_reader(self):
        # Obtain camera dimensions
        cam = self.camera_read_external()
        height, width, channels = cam.shape
        dim = (width, height)
        
        # Load image
        path = "images/pizza.jpg"
        BRG_img = cv2.imread(path,1)
        img = cv2.cvtColor(BRG_img, cv2.COLOR_BGR2RGB)
        
        # Resize image to camera dimensions
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        
        
        # print(height, width)
        self.image_to_display(img)
    
    def run_pose_estimation(self):
        pTime = 0
        detector = PoseDetector()
        i = 0
        
        L_timestamp = []
        R_timestamp = []
        
        while self.step(self.timeStep) != -1 and i < 200:
            print(i)
            
            img = self.camera_read_external()
            img = detector.findPose(img)
            h, w, c = img.shape
            
            lmList = detector.getPosition(img)
            
            
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
            
            # If posture is visible
            if lmList:
                i = i + 1
                # self.test(lmList.get(11), lmList.get(13), lmList.get(15))
                # self.move_left_elbow(lmList.get(11), lmList.get(13), lmList.get(15))
                # self.move_right_elbow(lmList.get(12), lmList.get(14), lmList.get(16))
                
                L = self.move_left_elbow(lmList.get(11), lmList.get(13), lmList.get(15))
                R = self.move_right_elbow(lmList.get(12), lmList.get(14), lmList.get(16))
                print(L)
                
                
                face_center = self.compute_face_center(lmList.get(0).x, lmList.get(0).y, img)
                self.look_at(face_center, velocity=1.)
            
            L_timestamp.append(L)
            R_timestamp.append(R)
                
                
                
            
            cv2.putText(img, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            self.image_to_display(img)
        if self.ext_camera:
            self.cameraExt.release()
            
    
        textfile = open("random_movements.txt", "w")
        for timestamp in range(30,len(L_timestamp)):
            for element in L_timestamp[timestamp]:
                textfile.write(str(element) + ",")
            textfile.write("\n")
            for element in R_timestamp[timestamp]:
                textfile.write(str(element) + ",")
            textfile.write("\n")
        textfile.close()
            
            
            
            
    def move_left_elbow(self, lm_shoulder, lm_elbow, lm_wrist): 
        pi = np.pi
        
        self.LShoulderRoll = self.getDevice('LShoulderRoll')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')
        self.LElbowRoll = self.getDevice('LElbowRoll')
        self.LElbowYaw = self.getDevice('LElbowYaw')
        
        self.LShoulderRoll.setVelocity(10.)
        self.LShoulderPitch.setVelocity(10.)
        self.LElbowRoll.setVelocity(10.)
        self.LElbowYaw.setVelocity(10.)
        
        
        x_sh = lm_shoulder.x
        y_sh = lm_shoulder.y
        z_sh = lm_shoulder.z
        
        x_el = lm_elbow.x
        y_el = lm_elbow.y
        z_el = lm_elbow.z
        
        x_wr = lm_wrist.x
        y_wr = lm_wrist.y
        z_wr = lm_wrist.z
        
        
        #Angle in radians, calculated clockwise starting from
        #Rightmost point in the circle 
        sh_roll = math.atan2(x_el - x_sh, y_el-y_sh,)
        converted_sh_roll = np.interp(sh_roll, [0., pi/2], [0., 1.32])
        if converted_sh_roll < 1.32 and sh_roll > 0:
            self.LShoulderRoll.setPosition(converted_sh_roll)
        
        
        el_relative = z_sh - z_el
        wr_relative = z_el - z_wr
        
        converted_el_relative = np.interp(el_relative, [0,1], [1.5,0.5])
        self.LShoulderPitch.setPosition(converted_el_relative)
        
        
        # Elbow Yaw (for wave motion) and basic hand movement in front
        # of torso
        if y_wr < y_el and y_wr < y_sh:
            self.LShoulderRoll.setPosition(1.32)
            self.LElbowYaw.setPosition(-2.05)
                 
        elif x_wr < x_el and y_wr > y_el:
            self.LElbowYaw.setPosition(-1.5)
            self.LElbowRoll.setPosition(-1.5)
            
        else:
            self.LElbowYaw.setPosition(0)   
            
        
        
        el_roll = math.atan2(x_wr - x_el, y_wr-y_el,)
        converted_el_roll = np.interp(el_roll, [0., 3.], [-1.52, 0.])

        #In current testing there seemed to be an offset
        offset = 0.5
        if -1.53 <converted_el_roll < 0:
            self.LElbowRoll.setPosition(converted_el_roll+offset)
        
        return [0, x_sh, y_sh, z_sh, x_el, y_el, z_el, x_wr, y_wr, z_wr]
        
    
    
    # Experimental method 
    def move_right_elbow(self, lm_shoulder, lm_elbow, lm_wrist): 
        pi = np.pi
             
             
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.RShoulderPitch = self.getDevice('RShoulderPitch')
        self.RElbowRoll = self.getDevice('RElbowRoll')
        self.RElbowYaw = self.getDevice('RElbowYaw')
        
        self.RShoulderRoll.setVelocity(10.)
        self.RShoulderPitch.setVelocity(10.)
        self.RElbowRoll.setVelocity(10.)
        self.RElbowYaw.setVelocity(10.)
        
        
        
        x_sh = lm_shoulder.x
        y_sh = lm_shoulder.y
        z_sh = lm_shoulder.z
        
        x_el = lm_elbow.x
        y_el = lm_elbow.y
        z_el = lm_elbow.z
        
        x_wr = lm_wrist.x
        y_wr = lm_wrist.y
        z_wr = lm_wrist.z
        
        
        #Angle in radians, calculated clockwise starting from
        #Rightmost point in the circle 
        sh_roll = math.atan2(x_el - x_sh, y_el-y_sh,)
        converted_sh_roll = np.interp(sh_roll, [-pi/2, 0.], [-1.32, 0.])
        if converted_sh_roll > -1.32 and sh_roll < 0:
            self.RShoulderRoll.setPosition(converted_sh_roll)
        
        el_relative = z_sh - z_el
        wr_relative = z_el - z_wr
        
        converted_el_relative = np.interp(el_relative, [0,1], [1.5,0.5])
        self.RShoulderPitch.setPosition(converted_el_relative)
        
        
        # Elbow Yaw (for wave motion) and basic hand movement in front
        # of torso
        if y_wr < y_el and y_wr < y_sh:
            self.RShoulderRoll.setPosition(-1.32)
            self.RElbowYaw.setPosition(2.05)
                 
        elif x_wr > x_el and y_wr > y_el:
            self.RElbowYaw.setPosition(1.5)
            self.RElbowRoll.setPosition(0)
            
        else:
            self.RElbowYaw.setPosition(0)  
        
        
        # v1 = [x_el, y_el, z_el]
        # v2 = [x_sh,y_sh, z_sh]      
        # sh_roll = math.atan2(z_el-z_sh, x_el - x_sh)
        # converted_sh_roll = (sh_roll/pi)*(2.09+2.09)-2.09
        # converted_sh_roll = 2.09 - converted_sh_roll
        
        el_roll = math.atan2(x_wr - x_el, y_wr-y_el,)
        converted_el_roll = np.interp(el_roll, [-3., 0.], [0., 1.52])
        #In current testing there seemed to be an offset
        offset = -0.5
        if 0 <converted_el_roll < 1.53:
            # print(converted_el_roll)
            self.RElbowRoll.setPosition(converted_el_roll+offset)
        
        
        return [1, x_sh, y_sh, z_sh, x_el, y_el, z_el, x_wr, y_wr, z_wr]
        
        
            
    def compute_face_center(self, fx, fy, img): 
        iy, ix, _ = img.shape
        
        # map the face center coordinates to the allowed actuators' ranges 
        # face_center = (fx/ix - 0.5) * 2.0855, (fy/iy - 0.5) * 0.5
        extcam_yaw = np.interp(fx/ix, [0., 1.], [2.09, -2.09])
        extcam_pitch = np.interp(fy/iy, [0., 1.], [-0.67, 0.51])
        # divide yaw value by 2, otherwise the face following of the robot looks more unnatural
        face_center = extcam_yaw / 2, extcam_pitch
        return face_center
    
    def look_at(self, pos, velocity=1.):
        """Instructs the robot to look at the position pos (x,y)
        pos[0]: must be a float value in the allowed range [-2.09,2.09] for the yaw actuator
        pos[1]: must be a float value in the allowed range [-0.67,0.51] for the pitch actuator
        """
        self.head_pitch.setVelocity(velocity)
        self.head_yaw.setVelocity(velocity)
        
        x_yaw, y_pitch = float(pos[0]), float(pos[1])
        self.head_yaw.setPosition(x_yaw)
        self.head_pitch.setPosition(y_pitch)


       
##############
#### MAIN ####    
############## 
# create the Robot instance and run the controller
robot = MyRobot(ext_camera_flag = True)
# robot.run_keyboard()
# robot.run_face_follower()
robot.run_image_reader()
robot.run_pose_estimation()
