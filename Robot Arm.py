import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

def run():
    length1  = 62
    length2  = 93
    length3  = 93
    length4  = 62
    length5  = 93
    length6  = 61
    initial_theta = -0
    final_theta   =  1

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
    initial_matrix_servo_1 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    initital_point = Point(initial_matrix_servo_1)
    final_position_servo_1 = translation_z(point=initital_point, z=length_1)
    print(final_position_servo_1.matrix)

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

def translation_z(point, z):
    result_matrix = np.copy(point.matrix)
    result_matrix[0, 3] = point.matrix[0, 3] + point.matrix[0, 2] * z
    result_matrix[1, 3] = point.matrix[1, 3] + point.matrix[1, 2] * z
    result_matrix[2, 3] = point.matrix[2, 3] + point.matrix[2, 2] * z
    result_matrix[3, 3] = point.matrix[3, 3] + point.matrix[3, 2] * z
    return Point(matrix=result_matrix)
    

if __name__ == '__main__':
    run()
