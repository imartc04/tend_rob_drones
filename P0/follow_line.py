from GUI import GUI
from HAL import HAL
import cv2
import queue
from functools import reduce
import numpy as np
import time
import math
# Enter sequential code!



class PID:
    def __init__(self, f_Kp, f_Kd, f_Ki, f_err_buff_size = 10):
        self.Kp = f_Kp
        self.Kd = f_Kd
        self.Ki = f_Ki

        self.err_buffer = queue.Queue(maxsize = f_err_buff_size)
    

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
        



def isWhiteSurrounded(f_img, f_pos, f_min_surround = 3, f_window = 3):
    l_ctr = 0
    n_rows, n_cols = f_img.shape
    
    ret = False
    
    for i in range(max(0, f_pos[0]-f_window), min(n_rows, f_pos[0]+f_window)):
        for j in range(max(0, f_pos[1]-f_window), min(n_cols, f_pos[1]+f_window)):
            if f_img[i,j] == 255:
                l_ctr +=1

    if l_ctr >= f_min_surround:
        ret = True
      
      
    return ret

def gen_range(f_init, f_end):
    
    rang = None
    if f_init <= f_end:
        rang = range(f_init, f_end)
    else:
        rang = range(f_init, f_end, -1)
        
    return rang

def try_get_white_in_range(f_img,f_row_start, f_row_end, f_col_start, f_col_end):
    
    n_rows, n_cols = f_img.shape
   
    row_range = gen_range(f_row_start, f_row_end)
    col_range = gen_range(f_col_start, f_col_end)
    
    white_pixel = None
    for row in row_range:
        for col in col_range:
            if f_img[row, col] == 255:
                if isWhiteSurrounded(f_img, (row, col)):
                    white_pixel = (row, col)
        
        if white_pixel != None:
            break
    
    return white_pixel
    

def try_get_line_from_bin_img(f_img):
    
    
    ret = None
    
    #Search for bottom white pixels 
    n_rows, n_cols = f_img.shape
    
    #Seach bottom left white pixel
    white_bott_left = try_get_white_in_range(f_img, n_rows-1, 1, 0, n_cols)
    
    
    #Seach bottom rigth white pixel
    white_bott_right = try_get_white_in_range(f_img, n_rows-1, 1, n_cols-1, 1)
    
        
    #Obtain top left white
    white_top_left = try_get_white_in_range(f_img, 0, n_rows, 0, n_cols)
    

    if white_top_left != None and (white_bott_left != None or white_bott_right != None):
        
        bottom = None
        if white_bott_right != None and white_bott_left != None:
            #Obtain botom point
            bottom = ( (white_bott_right[0] + white_bott_left[0])//2,
            (white_bott_right[1] + white_bott_left[1])//2)
            
        elif white_bott_left != None and white_bott_right == None:
            bottom = white_bott_left
      
        elif white_bott_left == None and white_bott_right != None:
            bottom = white_bott_right
            
        else:
            None
        
        
        #Make line and obtain its offset from the image middle and slope
        centroid_err = 0.5 - ((white_top_left[1] + bottom[1] )/2.0)/float(n_cols)
        top_center_err = 0.5 - white_top_left[1]/float(n_cols)
        
        ret = {"top_center_err" : top_center_err, "centroid_err" : centroid_err}
    
    #print("ret value : ", ret)
    return ret
    
def try_obtain_line(f_img):
    
    # Convert the image to the RGB color space
    image_rgb = cv2.cvtColor(f_img, cv2.COLOR_BGR2RGB)
    
    # Define the lower and upper color thresholds
    lower_color = np.array([215, 38, 38])  # Lower color threshold (in RGB format)
    upper_color = np.array([219, 43, 43])  # Upper color threshold (in RGB format)
    
    # Create a mask by thresholding the image within the color range
    mask = cv2.inRange(image_rgb, lower_color, upper_color)
    
    # Create a binary image by applying the mask
    binary_image = np.zeros_like(mask)
    binary_image[mask > 0] = 255
    GUI.showImage(binary_image)
    
    return try_get_line_from_bin_img(binary_image)
  
#Create PID objects
pid_ops = PIDOps()
top_center_pid = PID(3.0, 0.0, 0.0)
centroid_pid = PID(1.2, 0.4, 0.02)

time_loop = time.time()

vel_max = 8.0
vel_min = 2.0
first_loop = True #Fist loop has delta time approx 0 ms

while True:
    # Enter iterative code!
    delta_time = time.time() - time_loop
    #print("delta_time = ", delta_time)
    time_loop = time.time()
    
    #Try to obtain system errors with perception
    img = HAL.getImage()
    
    err = try_obtain_line(img)
    
    if err != None and first_loop == False:
        #print("Errors : ", err)
            
        pid_ops.add_error(top_center_pid, err["top_center_err"])
        pid_ops.add_error(centroid_pid, err["centroid_err"])
        
        #Calcule PID outputs
        top_center_out =  pid_ops.calcule_out(top_center_pid, delta_time, 4.0)
        centroid_out =  pid_ops.calcule_out(centroid_pid, delta_time, 2.0)
        
        print("PID outputs, top_center_out= ", top_center_out, " centroid_out= ", centroid_out)
     
        HAL.setV( vel_min) #max(vel_min, vel_max - 15.0* abs(top_center_out) - 15.0 * abs(centroid_out)) )
        HAL.setW(centroid_out) 
    
        #GUI.showImage(img)
    else:
        first_loop = False
    
    
    
    