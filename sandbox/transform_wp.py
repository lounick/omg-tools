import numpy as np
import pickle
import time

class TransformWP:
    def __init__(self, discretization):
        self.discretization = discretization
        self.angular_step = 2*np.pi/discretization
    
    def get_value(self, wpi, wpf, theta_i, theta_f):
        # Make WPi (0,0)
        wp = np.array([wpf[0]-wpi[0],wpf[1]-wpi[1]]) 
        wp_direction = self.get_step_from_angle(np.arctan2(wp[1],wp[0]))
        dx, dy = self.get_distance(wp)
        angular_offset = self.compute_angular_offset([dx,dy], wp)
        theta_i_bar =  self.get_angular_step(theta_i, angular_offset)
        theta_f_bar =  self.get_angular_step(theta_f,angular_offset)        
        return dx, dy, theta_i_bar, theta_f_bar

    def get_distance(self, wp):
        dx = np.maximum(np.abs(wp[0]),np.abs(wp[1])) 
        dy = np.minimum(np.abs(wp[0]),np.abs(wp[1]))
        return int(dx), int(dy)

    def get_angular_step(self, theta, wp_direction):
        if(theta > 0 ):
            theta += -wp_direction
            if (theta < 1):
                theta += self.discretization
        return theta 

    def get_step_from_angle(self,theta):
        if (theta < 0):
            theta += 2*np.pi
        step = theta/self.angular_step + 1    
        return int(step)

    def compute_angular_offset(self, wp_original, wp):
        theta_original = np.arctan2(wp_original[1],wp_original[0])
        theta_wp = np.arctan2(wp[1],wp[0])
        diff = np.abs(theta_wp - theta_original)
        return int(diff/self.angular_step)

#n_steps = 16
#dx, dy, theta_i_bar, theta_f_bar = transf.get_value([0,0], [-1,-1], 1, 3)
#a= 2

# open a file, where you stored the pickled data
file = open('cost8', 'rb')
# dump information to that file
cost = pickle.load(file)
# close the file
file.close()
n_steps = 8
transf = TransformWP(n_steps )

bl_wp = np.array([0,0])  # bottom left waypoint
ur_wp = np.array([9,9])  # upper right waypoint
size = [10,10] # nb of elements in the (x,y) axes
xx, yy = np.meshgrid(np.linspace(bl_wp[0], ur_wp[0], size[0]), 
                     np.linspace(bl_wp[1], ur_wp[1], size[1]) )

numel_WP = size[0]*size[1]  # number of waypoints in the grid
WP = np.array( [ xx.reshape(1,numel_WP)[0], yy.reshape(1,numel_WP)[0] ] )

tic = time.time()
for i in range(0, numel_WP):
    for f in range(0, numel_WP):
        if(i==f):
            continue       
        for theta_i in range(0, n_steps +1):
            for theta_f in range(0, n_steps +1):
                if (theta_i == 0 and i > 0) or (theta_f == 0 and f < numel_WP-1) or \
                    (i==0 and theta_i > 0) or (f==numel_WP-1 and theta_f > 0):
                    mycost = 9999.0 #np.norm(WP[:,i] - WP[:,f]) # high cost here
                else:
                    dx, dy, thetai, thetaf = transf.get_value(WP[:,i], WP[:,f], theta_i, theta_f)
                    if (dx < 10 and dy <10):
                        mycost = cost[dx,dy,thetai,thetaf]  
                    elif (f == numel_WP-1):
                        dxx = int(np.ceil(2*dx/np.sqrt(dx*dx+dy*dy)))
                        dyy = int(np.ceil(2*dy/np.sqrt(dx*dx+dy*dy)))
                        mycost = cost[dxx,dyy,thetai,thetaf] + np.linalg.norm(WP[:,i] - WP[:,f])
                print i, f, theta_i, theta_f, mycost
                        

