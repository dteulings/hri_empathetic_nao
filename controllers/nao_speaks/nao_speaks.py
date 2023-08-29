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
import sys
from PoseModule import PoseDetector
from controller import Speaker
from settings import intro, input_prompt, goodbye, images


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
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timeStep)
        self.previous_msg = ''

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
            
    def run_pose_estimation(self):
        pTime = 0
        detector = PoseDetector()
        while self.step(self.timeStep) != -1:
            img = self.camera_read_external()
            img = detector.findPose(img)
            h, w, c = img.shape
            
            lmList = detector.getPosition(img)
            
            
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
            
            # ID12 is left elbow
            if lmList:
                self.move_left_elbow(lmList.get(11), lmList.get(13), lmList.get(15))
                self.move_right_elbow(lmList.get(12), lmList.get(14), lmList.get(16))
            
            cv2.putText(img, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            self.image_to_display(img)
        if self.ext_camera:
            self.cameraExt.release()
            
    # Experimental method 
    def move_left_elbow(self, lm_shoulder, lm_elbow, lm_wrist): 
        pi = np.pi
             
             
        self.LShoulderRoll = self.getDevice('LShoulderRoll')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')
        self.LElbowRoll = self.getDevice('LElbowRoll')
        
        self.LShoulderRoll.setVelocity(5.)
        self.LShoulderPitch.setVelocity(5.)
        self.LElbowRoll.setVelocity(5.)
        
        
        
        offset = 10
        
        x_sh = lm_shoulder.x
        y_sh = lm_shoulder.y
        z_sh = lm_shoulder.z + offset
        
        x_el = lm_elbow.x
        y_el = lm_elbow.y
        z_el = lm_elbow.z + offset
        
        x_wr = lm_wrist.x
        y_wr = lm_wrist.y
        z_wr = lm_wrist.z + offset
        
        
        #Angle in radians, calculated clockwise starting from
        #Rightmost point in the circle 
        sh_roll = math.atan2(x_el - x_sh, y_el-y_sh,)
        #I assume the roll interval to be between 0 and 1.33
        converted_sh_roll = (2*sh_roll/pi)*1.33
        if converted_sh_roll < 1.32 and sh_roll > 0:
            self.LShoulderRoll.setPosition(converted_sh_roll)
        
        
        #Roll doesn't work very well, since z seems unrealiable
        v1 = [x_el, y_el, z_el]
        v2 = [x_sh,y_sh, z_sh]      
        sh_roll = math.atan2(z_el-z_sh, x_el - x_sh)
        converted_sh_roll = (sh_roll/pi)*(2.09+2.09)-2.09
        converted_sh_roll = 2.09 - converted_sh_roll
        
        # min_filter = -0.5
        # max_filter = 0.5
        # if -2.09 < converted_sh_roll < min_filter or max_filter < converted_sh_roll < 2.09:
            # print('hello')
            # self.LShoulderPitch.setPosition(converted_sh_roll)
        
        el_roll = math.atan2(x_wr - x_el, y_wr-y_el,)
        # #I assume the roll interval to be between -1.54 and 0
        converted_el_roll = -abs((el_roll)/3*1.54 - 1.54)
        #In current testing there seemed to be an offset
        offset = 0.5
        if -1.53 <converted_el_roll < 0:
            print(converted_el_roll)
            self.LElbowRoll.setPosition(converted_el_roll+offset)
        
        
        
    
    
    # Experimental method 
    def move_right_elbow(self, lm_shoulder, lm_elbow, lm_wrist): 
        pi = np.pi
             
             
        self.ShoulderRoll = self.getDevice('RShoulderRoll')
        self.ShoulderPitch = self.getDevice('RShoulderPitch')
        self.ElbowRoll = self.getDevice('RElbowRoll')
        
        self.ShoulderRoll.setVelocity(5.)
        self.ShoulderPitch.setVelocity(5.)
        self.ElbowRoll.setVelocity(5.)
        
        
        
        offset = 10
        
        x_sh = lm_shoulder.x
        y_sh = lm_shoulder.y
        z_sh = lm_shoulder.z + offset
        
        x_el = lm_elbow.x
        y_el = lm_elbow.y
        z_el = lm_elbow.z + offset
        
        x_wr = lm_wrist.x
        y_wr = lm_wrist.y
        z_wr = lm_wrist.z + offset
        
        
        #Angle in radians, calculated clockwise starting from
        #Rightmost point in the circle 
        sh_roll = math.atan2(x_el - x_sh, y_el-y_sh,)
        #I assume the roll interval to be between 0 and 1.33
        converted_sh_roll = (2*sh_roll/pi)*1.33
        if converted_sh_roll > -1.32 and sh_roll < 0:
            self.ShoulderRoll.setPosition(converted_sh_roll)
        
        
        #Roll doesn't work very well, since z seems unrealiable
        # v1 = [x_el, y_el, z_el]
        # v2 = [x_sh,y_sh, z_sh]      
        # sh_roll = math.atan2(z_el-z_sh, x_el - x_sh) # original
        # sh_roll = math.atan2(x_el-x_sh, y_el - y_sh)
        # converted_sh_roll = (sh_roll/pi)*(2.09+2.09)-2.09
        # converted_sh_roll = 2.09 - converted_sh_roll
        # print(sh_roll, converted_sh_roll)
        # if converted_sh_roll < (2.09) and converted_sh_roll > -2.09:
            # self.ShoulderPitch.setPosition(converted_sh_roll)
            
        
        # el_roll = math.atan2(x_wr - x_el, y_wr-y_el,)
        # print(el_roll)
        #I assume the roll interval to be between -1.54 and 0
        # converted_sh_roll = (el_roll-)*1.54 - 1.54
        # if converted_sh_roll < 0 and sh_roll > -1.53:
            # self.LShoulderRoll.setPosition(converted_sh_roll)
        
        
        
        
            
    def compute_face_center(self, face, img):
        x,y,w,h = face
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)  # draw a rectangle around the detected face
        
        face_center = x+int(w/2), y+int(h/2)
        cv2.circle(img, face_center, 2, (255,0,0), 2)  # draw a circle/dot at the center of the detected face

        iy, ix, _ = img.shape
        fx, fy = face_center
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
        
    def check_keyboard(self):
        """Offers options to control the experiment via keyboard
        """
        k = self.keyboard.getKey()
        msg = ''
        if k == ord('Y'):
            msg = 'Set experiment to condition A.'
            if msg != self.previous_msg:
                print(msg)
                self.condition = True
                
        if k == ord('N'):
            msg = 'Set experiment to condition B.'
            if msg != self.previous_msg:
                print(msg)
                self.condition = False       
                
        if k == ord('A'):
            msg = 'Abort the experiment..'
            if msg != self.previous_msg:
                print(msg)
                self.total_state = 'abort'
                self.previous_msg = msg
    
    def speak(self):
        ## https://cyberbotics.com/doc/reference/speaker?tab-language=python
        
        ## might need to install the program espeak: sudo apt-get install espeak
        ## https://cyberbotics.com/doc/guide/robotis-op2#speaker
        #spk = self.getDevice("Speaker")
        # speaker = speaker("speaker")
        
        
        speaker = Speaker("speaker")
        print(speaker.getEngine())
        print(speaker.getLanguage())
        # text = 'Hello! Using the text-to-speech of the Speaker device, I can speak 6 different languages: English with US or UK accent, German, Spanish, French and Italian. Using tags I can modulate my speech, like for example change <prosody pitch="+16.8st">the pitch of my voice</prosody>, <prosody pitch="-15st">and speak with a very low pitch</prosody>. <prosody rate="0.5">And I can change the speed</prosody><prosody rate="1.5">at which I speak</prosody>. I can also <prosody volume="20">adjust the volume of my voice</prosody>.'
        
        lst_msg = ""
        while self.step(self.timeStep) != -1:
            k = self.keyboard.getKey()
            if not speaker.isSpeaking():
                if k == ord('I') and lst_msg != intro:
                    print("speak intro")
                    speaker.speak(intro, volume=1)
                    lst_msg = intro
                elif k == ord('B') and lst_msg != goodbye:
                    print("speak goodbye")
                    speaker.speak(goodbye, volume=1)
                    lst_msg = goodbye
                elif k == ord('P') and lst_msg != input_prompt:
                    print("speak prompt")
                    speaker.speak(input_prompt, 1)
                    lst_msg = input_prompt
                elif k in range(48,58) and lst_msg != k:
                    print(f"speak image {k-48}")
                    speaker.speak(images[str(k-48)]["text"], 1)
                    lst_msg = k
                    
            else:
                lst_msg = ''


       
##############
#### MAIN ####    
############## 
# create the Robot instance and run the controller
robot = MyRobot(ext_camera_flag = True)
# robot.run_keyboard()
# robot.run_face_follower()
# robot.run_pose_estimation()
robot.speak()
