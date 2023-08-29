"""
MAIN controller
"""
# webots imports
from controller import Robot, Speaker, Keyboard, Device
# local imports
import threading
import warnings
from nao import MyRobot
from settings import intro, input_prompt, goodbye, images


warnings.filterwarnings("ignore", "WARNING")

class Experiment(MyRobot):
    def __init__(self):
        super(Experiment, self).__init__(ext_camera_flag=True)
        self.timeStep = int(self.getBasicTimeStep())
        
        print("Nao initialized")
        self.speaker = Speaker("speaker")
        print("Speaker initialized")
        
        # states
        self.total_state = "calibration" # states: calibration, experiment A, experiment B, end, pause, abort
        self.exp_state = 0 # states: 0=start, 1=describe, 2=listen, >2=end
        
        self.step_index = 0 # Keeps track of the step-index during NAO's pre-recorded movements
        self.previous_recoding = 1 # Keeps track of the previously displayed movement set
        self.L_timestamps = []
        self.R_timestamps = []
        
        self.condition = None
        
        self.cur_img = 0
        self.n_imgs = len(images.keys())
        print("n images: ", self.n_imgs)
        self.last_img_read = None
        
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timeStep)
        self.previous_msg = ''
        # self.keyboard = self.getKeyboard()
        
    def speak(self, text, volume=1):  
        text = "<prosody rate='0.95'>" + text + "</prosody>" 
        move_head = threading.Thread(target=self.speaker.speak, args=(text, volume))
        move_head.start()      
               
    def isSpeaking(self):
        return self.speaker.isSpeaking()
        
    def check_keyboard(self):
        """Offers options to control the experiment via keyboard
        """
        k = self.keyboard.getKey()
        msg = ''
        cond = self.condition
        if k == ord('Y') or k == ord('A'):
            msg = 'Set experiment to condition A.'
            if msg != self.previous_msg and (cond == 'B' or cond == None):
                print(msg)
                self.condition = 'A'
                self.previous_msg = msg
                
        if k == ord('N') or k == ord('B'):
            msg = 'Set experiment to condition B.'
            if msg != self.previous_msg and (cond == 'A' or cond == None):
                print(msg)
                self.condition = 'B'   
                self.previous_msg = msg
                
        elif k == ord('Z'):
            msg = 'Run experiment in condition A.'
            if msg != self.previous_msg:
                print(msg)
                self.total_state = 'experiment'
                self.condition = 'A'
                self.exp_state = 0
                self.previous_msg = msg
                
        elif k == ord('X'):
            msg = 'Run experiment in condition B.'
            if msg != self.previous_msg:
                print(msg)
                self.total_state = 'experiment'
                self.condition = 'B'
                self.exp_state = 0
                self.previous_msg = msg       
        
        elif k == ord('F'):
            msg = 'Set state to end.'
            if msg != self.previous_msg:
                print(msg)
                self.total_state = 'end'
                self.previous_msg = msg
                
        elif k == ord('P'):
            msg = 'Set state to pause.'
            if msg != self.previous_msg:
                print(msg)
                self.total_state = 'pause'
                self.previous_msg = msg
                
        elif k == ord('C'):
            msg = 'Set state to calibration'
            if msg != self.previous_msg:
                print(msg)
                self.total_state = 'calibration'
                self.previous_msg = msg
                
        elif k-48 in range(0,10):
            new_img = k-48
            if self.total_state == 'experiment':
                new_img -= 1
            msg = f"The next image will be image {k-48}."
            if self.previous_msg != msg:
                self.cur_img = new_img  # set the current (rather next) image to an index in 0-9
                print(msg)
                self.previous_msg = msg
        self.k = k
    
    def read_recorded_movements(self):
        
        if self.previous_recoding == 1:
            f = open("random_movements_2.txt", "r")
            self.previous_recording = 2
        else:
            f = open("random_movements_1.txt", "r")
            self.previous_recording = 1
        
        timestamps = f.read().split('\n')
        L_timestamps = []
        R_timestamps = []
        del timestamps[-1]
        for list in timestamps:
            split_list = list.split(',')
            del split_list[-1]
            split_list = [float(x) for x in split_list]
            # 0 == left, 1 == right
            if split_list[0] == 0: 
                L_timestamps.append(split_list)
            else:
                R_timestamps.append(split_list)
        f.close()
        
        return L_timestamps, R_timestamps
            
    
    def calibrate(self):
        """ The step in which the experimenter checks whether the Nao looks 
             at the participant and estimates the pose (correct distance?)
             
            :Return: True, if everything works fine
        """
        # look at face center
        self.look_at_face(display=True)
        
        # estimate pose
        
        # press 'Enter' to confirm calibration
        k = self.k
        if k == 4 and (self.condition == 'A' or self.condition == 'B'):
            print("Calibration complete.")
            print("Starting the experiment..")
            self.speak(intro)
            self.total_state = 'experiment'
            self.exp_state = 0
            self.read_img(str(self.cur_img))
        elif k != -1:
            if self.condition is None:
                msg = "Set the condition of the experiment first (press Y or N)"
                if self.previous_msg != msg:
                    print(msg)
                    self.previous_msg = msg
            else:
                cond = 'A' if self.condition == 'A' else 'B'            
                msg = f"Condition {cond} is set. Press 'ENTER' to confirm calibration"
                if self.previous_msg != msg:
                    print(msg)
                    self.previous_msg = msg
        
        
    def run(self):
        while self.step(self.timeStep) != -1:
            self.check_keyboard()
            
            state = self.total_state
            self.step_index = (self.step_index + 1) % 170
            # calibrate
            if state == "calibration":
                self.calibrate()
            
            # start the experimental loop with condition A
            # Mirroring is True
            elif state == "experiment" and self.condition == 'A':
                self.main_loop(True)
           
            # start the experimental loop with condition B
            # Mirroring is False
            elif state == "experiment" and self.condition == 'B':
                self.main_loop(False)
                
            # The simulation is about to end. Nao says good-bye.
            elif state == "end":
                if not self.isSpeaking():  # maybe this step is not necessary
                    print("Simulation ended")
                    break
            self.k = -1  # keyboard clean-up
            
            
        # clean-up
        if self.ext_camera:
            self.cameraExt.release()
            
    def read_img(self, img_key):
        path = images[img_key]["path"]
        if path != self.last_img_read:
            self.last_img_read = path
            self.run_image_reader(path)
        
    def main_loop(self, mirroring):
        self.look_at_face(display=False)
        
        img_key = str(self.cur_img)      
        # show image
        if self.exp_state == 0:
            # Nao descibes a picture
            if not self.isSpeaking():
                self.read_img(img_key)
                print("Current image:", img_key)
                self.speak(images[img_key]["text"])
                # switch to 'describe' experiment state
                self.exp_state = 1
        
        # while Nao speaks, do nothing
        # if Nao does not speak anymore, go to the next state (listen)                
        elif self.exp_state == 1:  # 1: description-state
            index = self.step_index
            
            if not self.isSpeaking():
                self.speak(input_prompt)
                self.exp_state = 2  # switch to 'listen' 
                
                if self.cur_img < self.n_imgs-1:   
                    self.L_timestamps, self.R_timestamps = self.read_recorded_movements()
                    print("Press ENTER to move to the next picture.")
                else:
                    print("Press ENTER to end the experiment.")            
                
        # Nao asks for the participants description
        elif self.exp_state == 2:
               
            ### Nao does its thing
            if self.total_state == "experiment" and self.condition == 'A':
                self.pose_estimation()
            
            elif self.total_state == "experiment" and self.condition == 'B':
                index = self.step_index
                self.recorded_motion(self.L_timestamps[index], self.R_timestamps[index])
            
            # Press ENTER to confirm that the participant finished their description
            if self.k == 4:  # 4 is ENTER
                self.cur_img += 1
                if self.cur_img < self.n_imgs:
                    self.exp_state = 0
                      
                else:
                    print("Nao says bye.")
                    self.speak(goodbye)
                    self.exp_state = -1
                    self.total_state = 'end'
            

##############
#### MAIN ####    
############## 
exp = Experiment()
exp.run()



