from GUI import GUI
from HAL import HAL
import cv2
import queue
import time
from functools import reduce
import math

# Enter sequential code!


def filter_img_thresh(f_img, f_thresh):
    thres, thres_img = cv2.threshold(gray_img, f_thresh ,255, cv2.THRESH_BINARY)
    return thres_img

def filter_img_range_thresh(f_img, f_thresh):

    # Apply thresholding to obtain a binary image with values above the lower threshold
    _, thresh1 = cv2.threshold(f_img,f_thresh[0],255, cv2.THRESH_BINARY)
    
    # Apply thresholding to obtain a binary image with values above the upper threshold
    _, thresh2 = cv2.threshold(f_img,f_thresh[1],255, cv2.THRESH_BINARY)
    
    return cv2.bitwise_xor(thresh1, thresh2)   

#Given a pixel location detect if it's surrounded by other white pixels
def isWhiteSurrounded(f_img, f_pos, f_min_surround = 3, f_window = 3):
    l_ctr = 0
    n_rows, n_cols = f_img.shape
    
    
    for i in range(max(0, f_pos[0]-f_window), min(n_rows, f_pos[0]+f_window)):
        for j in range(max(0, f_pos[1]-f_window), min(n_cols, f_pos[1]+f_window)):
            if f_img[i,j] == 255:
                l_ctr +=1
    
    
    if l_ctr >= f_min_surround:
        return True
  
  
    return False


#Try to detect a dense white point (White point ) in the zone of the image specified
def try_detect_dense_white(f_img, f_min_row, f_max_row):
    
    n_rows, n_cols = f_img.shape
    
    for i in range (f_min_row, f_max_row+1):
        for j in range(0, n_cols):
            if f_img[i,j] == 255:
                if isWhiteSurrounded(f_img, [i,j]):
                    return [i,j]
    
    return -1
    
    
#Given gray scale image try to detect line in the mid of the road
def try_observe_system(f_img):
    
    #Threshold the gray scale image
    road_img = filter_img_thresh(f_img, 85)
    GUI.showImage(road_img)
    
    n_rows, n_cols = road_img.shape
    
    #Try detect dense white point at image top 
    top_white = try_detect_dense_white(road_img, 0, 10)
    
    #Try detect dense white point at image bottom
    bottom_white = try_detect_dense_white(road_img, n_rows-11, n_rows-1)
    
    #Calculate heading and bias error
    head_err = 0.0
    if( top_white != -1 and bottom_white != -1 ):
    
        """
        Head error is the deviation over the X axis of the dron
        sin func is applied to make the metric independent of the num of pixels
        
        Drone frame : X axis points to front , Y axis point to left, Z upwards
        
        Yaw rate positive is a spin in counter clockwise direction view drone 
        from above
        
        """
        head_err = -math.sin((top_white[1] - bottom_white[1])/n_rows)
        bias_err =  0.5 - (float(bottom_white[1]) /n_cols) 
        
        print("bottom_white: ", bottom_white, " top_white: ", top_white, 
        ", mid_col : ", n_cols/2)
            
        return {"head": head_err, "bias_mid": bias_err}
        
    return -1



class PID:
    def __init__(self, f_Kp, f_Kd, f_Ki):
        self.Kp = f_Kp
        self.Kd = f_Kd
        self.Ki = f_Ki

        self.err_buffer = queue.Queue(maxsize = 10)
    

class PIDOps:
    def __init__(self):
        pass
    
    def calcule_out(self,f_pid, f_delta_time, f_max_abs= None):
        
        err_sum = reduce(lambda x, y: x + y, list(f_pid.err_buffer.queue), 0.0)
        
        #print("Err queue : ", f_pid.err_buffer.queue)
        #print("P term  : ", f_pid.Kp * f_pid.err_buffer.queue[0])
        
        ret = f_pid.Kp * f_pid.err_buffer.queue[-1] + \
        f_pid.Ki * err_sum * f_delta_time
        
        #Apply derivative term
        if f_pid.err_buffer.qsize() >= 2:
            ret += f_pid.Kd * (f_pid.err_buffer.queue[-1] - f_pid.err_buffer.queue[-2])/f_delta_time 
        
        if f_max_abs != None:
            if abs(ret)> f_max_abs:
                if ret > 0:
                    ret = f_max_abs
                else:
                    ret = -1.0 * f_max_abs
                    
        return ret
        
    def add_error(self,f_pid, f_err):
        if f_pid.err_buffer.full():
            f_pid.err_buffer.get()
            
        f_pid.err_buffer.put(f_err)


#Drone takeoff
print("#############################")
print("#############################")
print("############ NEW EXECUTION #################")

z_val = 1.2
init_pos = HAL.get_position()
init_yaw = HAL.get_yaw()

HAL.takeoff(z_val)
heading_pid = PID(2.5, 0.0, 0.0)
vel_pid = PID(2, 0.0, 0.05)

pid_ops = PIDOps()


#RGB lines : (122, 113, 72)
#img_rbg_thresh = list(map( filter_img_range_thresh, cv2.split(init_img), ((120,123),(110,114),(72,77
#))))
#print("type(img_rbg_thresh),  " ,type(img_rbg_thresh))
#img_rbg_thresh = cv2.merge(img_rbg_thresh)

#GUI.showImage(img_rbg_thresh)


time_point = time.time()
while True:
    #HAL.set_cmd_vel(0.0, 0.0, 0.0, 0.2)
    #Get image as gray scale
    gray_img = cv2.cvtColor(HAL.get_ventral_image(), cv2.COLOR_RGB2GRAY)
    #GUI.showImage(gray_img)
        
    #Process image and get angle respect to drone X axis and X offset
    control_err = try_observe_system(gray_img) 
    
    if control_err != -1:
        #Update controller errors
        time_diff = time.time() - time_point
        
        #print("time_diff : ", time_diff)
        
        time_point = time.time()
        
        print("Errors : ", control_err)
        
        pid_ops.add_error(heading_pid, control_err["head"])
        pid_ops.add_error(vel_pid, control_err["bias_mid"])
        
        print("Control out vy ", pid_ops.calcule_out(vel_pid,time_diff, 2.0))
        print("Control out heading ",
        pid_ops.calcule_out(heading_pid,time_diff))
        #Apply controller actions 
        
        vy = pid_ops.calcule_out(vel_pid,time_diff, 0.4)
        vx = max(0.4, 1.0 - 1.0 * abs(vy))
        HAL.set_cmd_mix(vx, vy, z_val,
        pid_ops.calcule_out(heading_pid,time_diff, 2.0))
    