"""HRI project Empathy

Student group:
Zuzanna Fendor s1008877
Steffen Ricklin s1009136
David Teulings s4576837
Loes Erven s4538757

Code used from (08-Dec-2021):
https://www.analyticsvidhya.com/blog/2021/05/pose-estimation-using-opencv/
"""
import cv2
import mediapipe as mp
import time

class PoseDetector:

    def __init__(self, mode = False, upBody = False, smooth=True, detectionCon = True, trackCon = 0.5):
        self.mode = mode
        self.upBody = upBody
        self.smooth = smooth
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode, self.upBody, self.smooth, self.detectionCon, self.trackCon)
    
    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
        
        return img
    
    def getPosition(self, img, draw=True):
        lmList = {}
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                lm.x = lm.x * w
                lm.y = lm.y * h
                lm.z = lm.z
                
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList[id] = lm
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        return lmList
    
def main():
    cap = cv2.VideoCapture('videos/a.mp4') #make VideoCapture(0) for webcam
    pTime = 0
    detector = PoseDetector()
    while True:
        success, img = cap.read()
        img = detector.findPose(img)
        lmList = detector.getPosition(img)
        
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        
        cv2.putText(img, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":

  main() 

