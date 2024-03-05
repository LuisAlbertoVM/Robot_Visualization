import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from enum import Enum

def run():
    length1  = 62
    length2  = 93
    length3  = 93
    length4  = 62
    length5  = 93
    length6  = 61
    initial_theta =  45
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
    #clear_3d_axes(ax)
    #plot_3d_lines(ax, x, y, z)
    #plt.pause(1)
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

class Plane:
    def __init__(self, point_a, point_b, point_c):
        self.point_a = point_a
        self.point_b = point_b
        self.point_c = point_c
        vector_1 = [point_b.x - point_a.x, point_b.y - point_a.y, point_b.z - point_a.z]
        vector_2 = [point_c.x - point_c.x, point_b.y - point_c.y, point_c.z - point_a.z]
        normal = [vector_1[1]*vector_2[2]-vector_1[2]*vector_2[1], vector_1[2]*vector_2[0]-vector_1[0]*vector_2[2], vector_1[0]*vector_2[1]-vector_1[1]*vector_2[0]]
        self.a = normal[0]
        self.b = normal[1]
        self.c = normal[2]
        self.d = -(normal[0] * point_a.x) - (normal[1]*point_a.y) - (normal[2]*point_a.z)

def distance_from_point_to_plane(point, plane):
    return abs(plane.a*point.x + plane.b*point.y + plane.c*point.z + plane.d)/math.sqrt(plane.a ** 2 + plane.b ** 2 + plane.c ** 2)

def translate_point_in_x_y_z(initial_point, x, y, z):
    translation_z(translation_y(translation_x(initial_point,x),y),z)

class Servo:
    def __init__(self,initial_point,servo_type,servo_orientation):
        self.initial_point = initial_point
        self.servo_type = servo_type
        self.servo_orientation = servo_orientation

        if self.servo_type == "RDS51150":
            self.width = 65
            self.heigth = 48
            self.depth = 30
        elif servo_type == "RDS3225":
            self.width = 65
            self.heigth = 48
            self.depth = 30

        if self.servo_orientation == "":
            pass
        elif "vertical":
            provisional_width = self.width
            provisional_heigth = self.heigth
            provisional_depth = self.depth
            self.width = provisional_heigth
            self.heigth = provisional_width
            self.depth = provisional_depth
        
        self.point_1 = translate_point_in_x_y_z(initial_point,           -(15/65) * self.width,  self.depth/2, 7)
        self.point_2 = translate_point_in_x_y_z(initial_point, self.width-(15/65) * self.width,  self.depth/2, 7)
        self.point_3 = translate_point_in_x_y_z(initial_point, self.width-(15/65) * self.width, -self.depth/2, 7)
        self.point_4 = translate_point_in_x_y_z(initial_point,           -(15/65) * self.width, -self.depth/2, 7)
        self.point_5 = translate_point_in_x_y_z(initial_point,           -(15/65) * self.width,  self.depth/2, self.heigth+7)
        self.point_6 = translate_point_in_x_y_z(initial_point, self.width-(15/65) * self.width,  self.depth/2, self.heigth+7)
        self.point_7 = translate_point_in_x_y_z(initial_point, self.width-(15/65) * self.width, -self.depth/2, self.heigth+7)
        self.point_8 = translate_point_in_x_y_z(initial_point,           -(15/65) * self.width, -self.depth/2, self.heigth+7)
        
        self.plane_down     = Plane(self.point_1, self.point_2, self.point_4)
        self.plane_up       = Plane(self.point_5, self.point_6, self.point_8)
        self.plane_forward  = Plane(self.point_1, self.point_2, self.point_5)
        self.plane_backward = Plane(self.point_4, self.point_3, self.point_8)
        self.plane_left     = Plane(self.point_1, self.point_5, self.point_4)
        self.plane_rigth    = Plane(self.point_3, self.point_2, self.point_7)

def is_point_inside_servo(point, servo):
    
    distance_base     = distance_from_point_to_plane(point, servo.plane_down)
    distance_top      = distance_from_point_to_plane(point, servo.plane_up)
    distance_forward  = distance_from_point_to_plane(point, servo.plane_forward)
    distance_backward = distance_from_point_to_plane(point, servo.plane_backward)
    distance_left     = distance_from_point_to_plane(point, servo.plane_left)
    distance_rigth    = distance_from_point_to_plane(point, servo.plane_rigth)

    is_between_base_top         = distance_base <= servo.heigth    and distance_top     <= servo.heigth
    is_between_backward_forward = distance_backward <= servo.depth and distance_forward <= servo.depth
    is_between_left_rigth       = distance_left     <= servo.width and distance_rigth   <= servo.width

    return is_between_base_top and is_between_backward_forward and is_between_left_rigth

def is_two_servo_collision(servo_1, servo_2):
    is_point_1_inside_servo_2 = is_point_inside_servo(servo_1.point_1, servo_2)
    is_point_2_inside_servo_2 = is_point_inside_servo(servo_1.point_2, servo_2)
    is_point_3_inside_servo_2 = is_point_inside_servo(servo_1.point_3, servo_2)
    is_point_4_inside_servo_2 = is_point_inside_servo(servo_1.point_4, servo_2)
    is_point_5_inside_servo_2 = is_point_inside_servo(servo_1.point_5, servo_2)
    is_point_6_inside_servo_2 = is_point_inside_servo(servo_1.point_6, servo_2)
    is_point_7_inside_servo_2 = is_point_inside_servo(servo_1.point_7, servo_2)
    is_point_8_inside_servo_2 = is_point_inside_servo(servo_1.point_8, servo_2)

    return is_point_1_inside_servo_2 or is_point_2_inside_servo_2 or is_point_3_inside_servo_2 or is_point_4_inside_servo_2 or is_point_5_inside_servo_2 or is_point_6_inside_servo_2 or is_point_7_inside_servo_2 or is_point_8_inside_servo_2

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
    result_matrix[:, 3] += result_matrix[:, 0] * x
    #result_matrix[0,3] = result_matrix[0,3] + result_matrix[0,0] * x
    #result_matrix[1,3] = result_matrix[1,3] + result_matrix[1,0] * x
    #result_matrix[2,3] = result_matrix[2,3] + result_matrix[2,0] * x
    #result_matrix[3,3] = result_matrix[3,3] + result_matrix[3,0] * x
    return Point(matrix=result_matrix)

def translation_y(point, y):
    result_matrix = np.copy(point.matrix)
    result_matrix[:, 3] += result_matrix[:, 1] * y
    #result_matrix[0,3] = result_matrix[0,3] + result_matrix[0,1] * y
    #result_matrix[1,3] = result_matrix[1,3] + result_matrix[1,1] * y
    #result_matrix[2,3] = result_matrix[2,3] + result_matrix[2,1] * y
    #result_matrix[3,3] = result_matrix[3,3] + result_matrix[3,1] * y
    return Point(matrix=result_matrix)

def translation_z(point, z):
    result_matrix = np.copy(point.matrix)
    result_matrix[:, 3] += result_matrix[:, 2] * z
    #result_matrix[0, 3] = point.matrix[0, 3] + point.matrix[0, 2] * z
    #result_matrix[1, 3] = point.matrix[1, 3] + point.matrix[1, 2] * z
    #result_matrix[2, 3] = point.matrix[2, 3] + point.matrix[2, 2] * z
    #result_matrix[3, 3] = point.matrix[3, 3] + point.matrix[3, 2] * z
    return Point(matrix=result_matrix)

def create_3d_axes():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_zlim(-200, 200)
    return ax

def plot_3d_lines(ax,x,y,z):
    ax.plot(x, y, z, c='b',marker='o')

def clear_3d_axes(ax):
    ax.clear()

if __name__ == '__main__':
    run()
