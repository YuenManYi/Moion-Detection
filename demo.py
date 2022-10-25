# This program is to count the number of raising hands. The task is under Academic and Learning domain in Applied Behavior Analysis.
# Existing prog is only available for single people. (But suitable for both left hand and right hand)
from math import radians
import cv2
import mediapipe as mp
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose



def calculate_angle(shoulder,elbow,wrist):
    shoulder = np.array(shoulder) #left or right shoulder
    elbow = np.array(elbow) #left or right elbow
    wrist = np.array(wrist) #left or right wrist

    radians = np.arctan2(wrist[1]-elbow[1], wrist[0]-elbow[0]) - np.arctan2(shoulder[1]-elbow[1], shoulder[0]-elbow[0])
    angle = np.abs(radians*180.0/np.pi)

    if angle >180.0:
        angle = 360-angle
        
    return angle 


def main():
    cap = cv2.VideoCapture(0)

    counter = 0
    leftstage = None
    rightstage = None

    #setup mediapipe instance ->accurate model/ tighter with detecion
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while cap.isOpened():
            ret, frame = cap.read()

            # Recolor image to RGB
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
        
            # Make detection
            results = pose.process(image)
        
            # Recolor back to BGR
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Extract landmarks
            try:
                landmarks = results.pose_landmarks.landmark

                # Get coordinates 
                left_shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
                left_elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
                left_wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]

                right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
                right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
                right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

                # Calculate angle
                leftAngle = calculate_angle(left_shoulder, left_elbow, left_wrist)
                rightAngle = calculate_angle(right_shoulder,right_elbow,right_wrist)

                # Visualize angle
                #cv2.putText(image, str(angle), tuple(np.multiply(elbow, [640, 480]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

                if leftAngle < 20:
                    leftstage = "down"
                elif leftAngle > 160 and leftstage == 'down' :
                    leftstage = "up"
                    counter += 1
                    print(counter)

                if rightAngle < 20:
                    rightstage = "down"
                elif rightAngle > 160 and rightstage == 'down' :
                    rightstage = "up"
                    counter += 1
                    print(counter)

            except:
                pass
            
            #Write Counter 
            cv2.rectangle(image, (0,0), (225,73), (0, 255, 0), -1)

            # Render detections
            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)   

            cv2.putText(image, 'Raise Hands:', (15,12), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1, cv2.LINE_AA)
            cv2.putText(image, str(counter), (15,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

            cv2.imshow('demo',image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        cap.relase()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()