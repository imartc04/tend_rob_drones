from GUI import GUI
from HAL import HAL
import time
import numpy as np
import cv2
from itertools import product


    
"""
Assumed a Haar good detection in the range [-30,30] degrees
one can rotate images to cover all posibiities by rotating
on angles 180,-90,90,-60,60


"""

def get_image_rotated(f_image, f_rot):
    height, width = f_image.shape[:2]

    # Calculate the rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), f_rot, 1)
    
    # Apply the rotation using warpAffine
    return cv2.warpAffine(f_image, rotation_matrix, (width, height))



def detect_faces(f_img, f_detector, f_rot_angles= [180.0, -90.0,90.0, -60.0, 60.0]):
    
    face_detected = False
    
    f_rot_angles = [0.0] + f_rot_angles
    
    #Convert image to gray scale
    frame_gray = cv2.cvtColor(f_img, cv2.COLOR_RGB2GRAY)
    frame_gray = cv2.equalizeHist(frame_gray)
    
    for i in f_rot_angles:
        
        img = get_image_rotated(f_img, i)
        
        #-- Detect faces
        faces = f_detector.detectMultiScale(img)
        
        for (x,y,w,h) in faces:
            center = (x + w/2, y + h/2)
            face_detected = True
            break
            print("Detectd face at px : ", x, " py : ", y)

        if face_detected:
            break
    
    return face_detected
    
def return_to_base(f_takeOffPoint):
    
    print("Returning to base..")
    
    #Go to takeoff point 
    HAL.set_cmd_pos(f_takeOffPoint[0], f_takeOffPoint[1], f_takeOffPoint[2], 0.0)
    
    #Give time to the drone to reach the point
    time.sleep(15)
    
    print("Landing...")
    #Land in base
    HAL.land()

def generate_grid(start_point, step, f_num_step_x, f_num_step_y):
    x_start, y_start = start_point
    x_step, y_step = step

    x_values = [x_start + x_step * i for i in range(f_num_step_x)]
    
    y_values = [y_start + y_step * j for j in range(f_num_step_y)]

    grid = list(product(x_values, y_values))
    return grid


#Create a face detector
#face_detector = cv2.CascadeClassifier(
#        'RoboticsAcademy/exercises/rescue_people/haarcascade_frontalface_default.xml')
face_detector = cv2.CascadeClassifier(
    '/RoboticsAcademy/exercises/static/exercises/rescue_people_react/haarcascade_frontalface_default.xml')

#Save init pose
init_pose = HAL.get_position()
init_yaw = HAL.get_yaw()

z_takeoff = init_pose[2]+5.0

z_search = 1.0

if HAL.get_landed_state() == 1:
    HAL.takeoff(z_takeoff)



"""
Boat UTM 430492 E 4459162 N
Surivors UTM 430532 E 5549132 N

E : 532-492 = 40 
N : 132-162 = -30

The drone is initialy aligned with reference frames

"""

dist = np.array([40.0 ,-30.0])
search_zone = (init_pose[0]+dist[0]+3.0, init_pose[1]+dist[1]+2.0)
HAL.set_cmd_pos(search_zone[0], search_zone[1], z_search, init_yaw)
#HAL.set_cmd_pos(init_pose[0]  ,init_pose[1] , z_takeoff, init_yaw)

#Give time to the drone to reach position
time.sleep(10)


#Generate grid of search points
step = (-3, -3)
n_step_x = 7
n_step_y = 6
grid = generate_grid(search_zone, step, n_step_x, n_step_y)

begin_time = time.time()
delta_time_return = 60.0


while True:

    if (time.time() - begin_time) >= delta_time_return:
        #Return to base 
        print("Timeout to return to base reached")
        return_to_base([init_pose[0],init_pose[1], z_takeoff])
    
    else:
        for point in grid:
            HAL.set_cmd_pos(point[0], point[1], z_search, init_yaw)
            
            #Give some time to the drone to reach the position
            time.sleep(2.0)
        
            img = HAL.get_ventral_image()
            GUI.showImage(img)
            
            if detect_faces(img, face_detector):
                print("Face detected at x = ", point[0], ", y = ", point[1])
        
    
    print("End of mision")
    return_to_base([init_pose[0],init_pose[1], z_takeoff])
    
    time.sleep(100000)
    
    
    
