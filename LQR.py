import numpy as np
np.set_printoptions(suppress=True)
import matplotlib.pyplot as plt
import math
from simple_pid import PID

##initial states
N = 1000  # iteration range
EPS = 1e-5  # iteration precision
Q = np.eye(3) * 1
R = np.eye(2) * 1

dt = 0.1  # sampling time
L = 2.84988  # vehicle wheelbase
v = 0  # initial speed
x_0 = 0  # initial x position
y_0 = 0  # initial y position
psi_0 = 0  # initial heading 

MAX_STEER = math.radians(45.0)  # maximum steering angle [rad]
MAX_DSTEER = math.radians(45.0)  # maximum steering speed [rad/s]
MAX_ACC = 2 # maximum accelerator [m/s]
pid = PID(1, 0.1, 0.05)
pid.output_limits = (-100, 100) 

def cal_Ricatti(A, B, Q, R):
    # Ricatti equation
    # Q is a semi-positive definite state weighting matrix, which is usually taken as a diagonal matrix; 
    # the larger the elements of the Q matrix, the hope that the tracking deviation can quickly approach zero;
    # R is a positive definite control weighting matrix, 
    # and the larger elements of the R matrix mean that the control input is expected to be as small as possible.
    
    # iteration initial value
    Qf = Q
    P = Qf
    # loop iteration
    for _ in range(N):
        P_ = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A
        if (abs(P_ - P).max() < EPS):
            break
        P = P_
    return P_

def LQR(robot_state, refer_path, s0, A, B, Q, R):
    # x is position and heading error
    x = robot_state[0:3] - refer_path[s0, 0:3]
    if x[2] > math.pi:
        x[2] = -(2 * math.pi - x[2])
    elif x[2] < -2 * math.pi:
        x[2] = -(x[2] + math.pi)

    P = cal_Ricatti(A, B, Q, R)
    K = -np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A
    u = K @ x
    u_star = u
    print(K, x, u)
    # steering wheel speed constraint
    if u_star[0, 1] > MAX_DSTEER:
        u_star[0, 1] = MAX_DSTEER
    if u_star[0, 1] < -MAX_DSTEER:
        u_star[0, 1] = -MAX_DSTEER

    return u_star[0, 1]

class KinematicModel:
    # Suppose the control quantity is steering angle delta_f and acceleration a
    def __init__(self, x, y, psi, v, L, dt):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v
        self.L = L
        self.dt = dt # Discretization

    def update_state(self, a, delta_f):
        self.x = self.x + self.v * math.cos(self.psi) * self.dt
        self.y = self.y + self.v * math.sin(self.psi) * self.dt
        self.psi = self.psi + self.v / self.L * math.tan(delta_f) * self.dt
        self.v = self.v + a * self.dt

    def state_space(self, ref_delta, ref_yaw):
        A = np.matrix([[1.0, 0.0, -self.v * self.dt * math.sin(ref_yaw)],
                       [0.0, 1.0, self.v * self.dt * math.cos(ref_yaw)],
                       [0.0, 0.0, 1.0]])

        B = np.matrix([[self.dt * math.cos(ref_yaw), 0],
                       [self.dt * math.sin(ref_yaw), 0],
                       [self.dt * math.tan(ref_delta) / self.L,
                        self.v * self.dt /(self.L * math.cos(ref_delta) * math.cos(ref_delta))]])
        return A, B

class MyReferencePath:
    def __init__(self):
        # simlulation PATH  
        ## new_waypoints_generator
        odom = np.loadtxt("odom.csv", delimiter = ",", dtype = float)       
        cur_wp = odom[0, :]
        new_wp = [] 
        new_wp.append(cur_wp)
        for i in range(np.size(odom, 0)):
            dis = ((cur_wp[0] - odom[i, 0]) ** 2 + (cur_wp[1] - odom[i, 1]) ** 2) ** 0.5
            if dis > 3:
                cur_wp = odom[i, :]
                new_wp.append(cur_wp)
        new_wp = np.array(new_wp)

        # refer_path includes 4 dimensions: position x, position y, tangent direction of track point yaw, curvature k
        self.refer_path = np.zeros((np.size(new_wp, 0), 5))
        self.refer_path[:, 0] = new_wp[:, 0]
        self.refer_path[:, 1] = new_wp[:, 1]

        # Calculate the first and second derivatives of the path point by difference to obtain the tangent direction and curvature
        for i in range(len(self.refer_path)):
            if i == 0:
                dx = self.refer_path[i + 1, 0] - self.refer_path[i, 0]
                dy = self.refer_path[i + 1, 1] - self.refer_path[i, 1]
                ddx = self.refer_path[2, 0] + self.refer_path[0, 0] - 2 * self.refer_path[1, 0]
                ddy = self.refer_path[2, 1] + self.refer_path[0, 1] - 2 * self.refer_path[1, 1]
            elif i == (len(self.refer_path) - 1):
                dx = self.refer_path[i, 0] - self.refer_path[i - 1, 0]
                dy = self.refer_path[i, 1] - self.refer_path[i - 1, 1]
                ddx = self.refer_path[i, 0] + self.refer_path[i - 2, 0] - 2 * self.refer_path[i - 1, 0]
                ddy = self.refer_path[i, 1] + self.refer_path[i - 2, 1] - 2 * self.refer_path[i - 1, 1]
            else:
                dx = self.refer_path[i + 1, 0] - self.refer_path[i, 0]
                dy = self.refer_path[i + 1, 1] - self.refer_path[i, 1]
                ddx = self.refer_path[i + 1, 0] + self.refer_path[i - 1, 0] - 2 * self.refer_path[i, 0]
                ddy = self.refer_path[i + 1, 1] + self.refer_path[i - 1, 1] - 2 * self.refer_path[i, 1]
            self.refer_path[i, 2] = math.atan2(dy, dx)  # tangent direction yaw
            self.refer_path[i, 3] = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))  # curvature k
            self.refer_path[i, 4] = i

    def calc_track_error(self, x, y, current_wp):
        # Calculate the tracking error
        # Find the closest target point on the reference trajectory
        d_x = [self.refer_path[i, 0] - x for i in range(current_wp, current_wp + 10)]
        d_y = [self.refer_path[i, 1] - y for i in range(current_wp, current_wp + 10)]
        d = [np.sqrt(d_x[i] ** 2 + d_y[i] ** 2) for i in range(len(d_x))]
        s = np.argmin(d) + current_wp # The target point s with the smallest distence

        yaw = self.refer_path[s, 2]
        k = self.refer_path[s, 3]
        delta = math.atan2(L * k, 1)

        feature_k = self.refer_path[s + 10, 3]
        feature_delta = math.atan2(L * feature_k, 1)

        return delta, yaw, s, feature_delta   

def main(args = None):
    reference_path = MyReferencePath()

    # initial KinematicModel
    ugv = KinematicModel(x_0, y_0, psi_0, v, L, dt)
    x_ = []
    y_ = []
    current_wp = 0

    # save image
    while(True):
        robot_state = np.zeros(4)
        robot_state[0] = ugv.x
        robot_state[1] = ugv.y
        robot_state[2] = ugv.psi
        robot_state[3] = ugv.v

        # calculate steering wheel u
        ref_delta, ref_yaw, ref_s, ref_F_delta = reference_path.calc_track_error(robot_state[0], robot_state[1], current_wp)
        current_wp = ref_s
        A, B = ugv.state_space(ref_delta, ref_yaw)
        delta_u = LQR(robot_state, reference_path.refer_path, ref_s, A, B, Q, R)
        
        update_delta = delta_u

        # steering wheel constraint
        if update_delta > MAX_STEER:
            update_delta = MAX_STEER
        if update_delta < -MAX_STEER:
            update_delta = -MAX_STEER

        # calculate thottle u
        if abs(ref_F_delta) < 0.01:        
            pid.setpoint = 16.667        
        else:
            pid.setpoint = 5.55
        throttle = pid(ugv.v)

        # update ugv state
        ugv.update_state(throttle, update_delta)

        x_.append(ugv.x)
        y_.append(ugv.y)

        # show image
        plt.cla()
        # plt.xlim([reference_path.refer_path[ref_s, 0] - 30, reference_path.refer_path[ref_s, 0] + 30])
        # plt.ylim([reference_path.refer_path[ref_s, 1] - 30, reference_path.refer_path[ref_s, 1] + 30])
        plt.plot(reference_path.refer_path[:, 0], reference_path.refer_path[:, 1], "-.b", linewidth = 1.0, label = "course")
        plt.plot(x_, y_, "-r", label = "trajectory")
        plt.plot(reference_path.refer_path[ref_s, 0], reference_path.refer_path[ref_s, 1], "go", label = "target")
        plt.grid(True)
        plt.pause(0.0000000001)

        # check last point
        if ref_s == len(reference_path.refer_path) - 1:
            break


if __name__ == '__main__':
    main()
