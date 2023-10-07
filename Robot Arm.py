import numpy as np
import matplotlib.pyplot as plt

def run():
    length1  = 62
    length2  = 93
    length3  = 93
    length4  = 62
    length5  = 93
    length6  = 61
    initial_theta = -90
    final_theta   =  90

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
    print(thetas)

class Point:
    def __init__(self, matrix):
        # Initialize the point
        self.matrix = matrix
        self.x = self.matrix[0][3]
        self.y = self.matrix[1][3]
        self.z = self.matrix[2][3]

def xRotation(point, theta):
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

    rotated_point = Point(result_matrix)

    return rotated_point


if __name__ == '__main__':
    run()
