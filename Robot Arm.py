import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D
from enum import Enum

def run():
    length1  = 62
    length2  = 93
    length3  = 93
    length4  = 62
    length5  = 93
    length6  = 61
    initial_theta =  90
    final_theta   =  91

    lengths = [length1, length2, length3, length4, length5, length6]
    for theta in range(initial_theta, final_theta):
        theta1 = theta * np.pi / 180
        theta2 = theta * np.pi / 180
        theta3 = theta * np.pi / 180
        theta4 = theta * np.pi / 180
        theta5 = theta * np.pi / 180
        theta6 = theta * np.pi / 180

        thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
        directKinematics(lengths=lengths, thetas=thetas)

def directKinematics(lengths, thetas):
    length_1 = lengths[0]
    length_2 = lengths[1]
    length_3 = lengths[2]
    length_4 = lengths[3]
    length_5 = lengths[4]
    length_6 = lengths[5]

    theta_1 = thetas[0]
    theta_2 = thetas[1]
    theta_3 = thetas[2]
    theta_4 = thetas[3]
    theta_5 = thetas[4]
    theta_6 = thetas[5]

    initial_matrix_servo_1 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    initital_position_servo_1 = Point(initial_matrix_servo_1)
    final_position_servo_1    = translation_z(point=rotation_z(initital_position_servo_1,theta_1), z=length_1)
    initital_position_servo_2 = final_position_servo_1
    final_position_servo_2    = translation_z(point=rotation_y(initital_position_servo_2,theta_2), z=length_2)
    initital_position_servo_3 = final_position_servo_2
    final_position_servo_3    = translation_z(point=rotation_y(initital_position_servo_3,theta_3), z=length_3)
    initital_position_servo_4 = final_position_servo_3
    final_position_servo_4    = translation_z(point=rotation_z(initital_position_servo_4,theta_4), z=length_4)
    initital_position_servo_5 = final_position_servo_4
    final_position_servo_5    = translation_z(point=rotation_y(initital_position_servo_5,theta_5), z=length_5)
    initital_position_servo_6 = final_position_servo_5
    final_position_servo_6    = translation_z(point=rotation_z(initital_position_servo_6,theta_6), z=length_6)

    x = [initital_position_servo_1.x, final_position_servo_1.x, final_position_servo_2.x, final_position_servo_3.x, final_position_servo_4.x, final_position_servo_5.x, final_position_servo_6.x]
    y = [initital_position_servo_1.y, final_position_servo_1.y, final_position_servo_2.y, final_position_servo_3.y, final_position_servo_4.y, final_position_servo_5.y, final_position_servo_6.y]
    z = [initital_position_servo_1.z, final_position_servo_1.z, final_position_servo_2.z, final_position_servo_3.z, final_position_servo_4.z, final_position_servo_5.z, final_position_servo_6.z]
    ax = create_3d_axes()
    plot_3d_lines(ax, x, y, z)
    plt.pause(5)
    clear_3d_axes(ax)
    plt.pause(5)
    plot_3d_lines(ax, x, y, z)
    plt.pause(5)
    clear_3d_axes(ax)


class ServoOrientarion(Enum):
    horizontal = 1
    vertical = 2

class ServoType(Enum):
    RDSS51150 = 1
    RDS3225 = 2

class Point:
    def __init__(self, matrix):
        # Initialize the point
        self.matrix = matrix
        self.x = self.matrix[0][3]
        self.y = self.matrix[1][3]
        self.z = self.matrix[2][3]

class Servo:
    def __init__(self,initial_point,servo_type,servo_orientation):
        self.initial_point = initial_point
        self.servo_type = servo_type
        self.servo_orientation = servo_orientation

        
        
        pass

def rotation_x(point, theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    matrix_a = point.matrix.copy()
    result_matrix = matrix_a.copy()

    result_matrix[0,1] =  matrix_a[0,1]*cos_theta + matrix_a[0,2]*sin_theta
    result_matrix[0,2] = -matrix_a[0,1]*sin_theta + matrix_a[0,2]*cos_theta
    result_matrix[1,1] =  matrix_a[1,1]*cos_theta + matrix_a[1,2]*sin_theta
    result_matrix[1,2] = -matrix_a[1,1]*sin_theta + matrix_a[1,2]*cos_theta
    result_matrix[2,1] =  matrix_a[2,1]*cos_theta + matrix_a[2,2]*sin_theta
    result_matrix[2,2] = -matrix_a[2,1]*sin_theta + matrix_a[2,2]*cos_theta
    result_matrix[3,1] =  matrix_a[3,1]*cos_theta + matrix_a[3,2]*sin_theta
    result_matrix[3,2] = -matrix_a[3,1]*sin_theta + matrix_a[3,2]*cos_theta

    return Point(result_matrix)

def rotation_y(point, theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    matrix_a = point.matrix.copy()
    result_matrix = matrix_a.copy()

    result_matrix[0,0] = matrix_a[0,0] * cos_theta - matrix_a[0,2] * sin_theta
    result_matrix[0,2] = matrix_a[0,0] * sin_theta + matrix_a[0,2] * cos_theta
    result_matrix[1,0] = matrix_a[1,0] * cos_theta - matrix_a[1,2] * sin_theta
    result_matrix[1,2] = matrix_a[1,0] * sin_theta + matrix_a[1,2] * cos_theta
    result_matrix[2,0] = matrix_a[2,0] * cos_theta - matrix_a[2,2] * sin_theta
    result_matrix[2,2] = matrix_a[2,0] * sin_theta + matrix_a[2,2] * cos_theta
    result_matrix[3,0] = matrix_a[3,0] * cos_theta - matrix_a[3,2] * sin_theta
    result_matrix[3,2] = matrix_a[3,0] * sin_theta + matrix_a[3,2] * cos_theta

    return Point(result_matrix)

def rotation_z(point, theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    matrix_a = point.matrix.copy()
    result_matrix = matrix_a.copy()

    result_matrix[0,0] =  matrix_a[0,0] * cos_theta + matrix_a[0,1] * sin_theta
    result_matrix[0,1] = -matrix_a[0,0] * sin_theta + matrix_a[0,1] * cos_theta
    result_matrix[1,0] =  matrix_a[1,0] * cos_theta + matrix_a[1,1] * sin_theta
    result_matrix[1,1] = -matrix_a[1,0] * sin_theta + matrix_a[1,1] * cos_theta
    result_matrix[2,0] =  matrix_a[2,0] * cos_theta + matrix_a[2,1] * sin_theta
    result_matrix[2,1] = -matrix_a[2,0] * sin_theta + matrix_a[2,1] * cos_theta
    result_matrix[3,0] =  matrix_a[3,0] * cos_theta + matrix_a[3,1] * sin_theta
    result_matrix[3,1] = -matrix_a[3,0] * sin_theta + matrix_a[3,1] * cos_theta

    return Point(result_matrix)

def translation_x(point, x):
    result_matrix = np.copy(point.matrix)
    result_matrix[0,3] = result_matrix[0,3] + result_matrix[0,0] * x
    result_matrix[1,3] = result_matrix[1,3] + result_matrix[1,0] * x
    result_matrix[2,3] = result_matrix[2,3] + result_matrix[2,0] * x
    result_matrix[3,3] = result_matrix[3,3] + result_matrix[3,0] * x
    return Point(matrix=result_matrix)

def translation_y(point, y):
    result_matrix = np.copy(point.matrix)
    result_matrix[0,3] = result_matrix[0,3] + result_matrix[0,1] * y
    result_matrix[1,3] = result_matrix[1,3] + result_matrix[1,1] * y
    result_matrix[2,3] = result_matrix[2,3] + result_matrix[2,1] * y
    result_matrix[3,3] = result_matrix[3,3] + result_matrix[3,1] * y
    return Point(matrix=result_matrix)

def translation_z(point, z):
    result_matrix = np.copy(point.matrix)
    result_matrix[0, 3] = point.matrix[0, 3] + point.matrix[0, 2] * z
    result_matrix[1, 3] = point.matrix[1, 3] + point.matrix[1, 2] * z
    result_matrix[2, 3] = point.matrix[2, 3] + point.matrix[2, 2] * z
    result_matrix[3, 3] = point.matrix[3, 3] + point.matrix[3, 2] * z
    return Point(matrix=result_matrix)

def create_3d_axes():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax

def plot_3d_lines(ax,x,y,z):
    ax.plot(x, y, z, c='b',marker='o')

def clear_3d_axes(ax):
    ax.clear()

if __name__ == '__main__':
    run()
