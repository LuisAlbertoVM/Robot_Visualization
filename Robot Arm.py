import numpy
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
        theta1 = theta * numpy.pi / 180
        theta2 = theta * numpy.pi / 180
        theta3 = theta * numpy.pi / 180
        theta4 = theta * numpy.pi / 180
        theta5 = theta * numpy.pi / 180
        theta6 = theta * numpy.pi / 180

        thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
        directKinematics(lengths=lengths, thetas=thetas)

def directKinematics(lengths, thetas):
    print(thetas)

def xRotation(point, theta):
    cos_theta = numpy.cos(theta)
    sin_theta = numpy.sin(theta)
    matrix_a = point.matrix
    result_matrix = matrix_a

    result_matrix[0,1] =  matrix_a[0,1]*cos_theta + matrix_a[0,2]*sin_theta
    result_matrix[0,2] = -matrix_a[0,1]*sin_theta + matrix_a[0,2]*cos_theta
    result_matrix[1,1] =  matrix_a[1,1]*cos_theta + matrix_a[1,2]*sin_theta
    result_matrix[1,2] = -matrix_a[1,1]*sin_theta + matrix_a[1,2]*cos_theta
    result_matrix[2,1] =  matrix_a[2,1]*cos_theta + matrix_a[2,2]*sin_theta
    result_matrix[2,2] = -matrix_a[2,1]*sin_theta + matrix_a[2,2]*cos_theta
    result_matrix[3,1] =  matrix_a[3,1]*cos_theta + matrix_a[3,2]*sin_theta
    result_matrix[3,2] = -matrix_a[3,1]*sin_theta + matrix_a[3,2]*cos_theta

    rotated_point.matrix = result_matrix
    rotated_point.x = result_matrix(1,4)
    rotated_point.y = result_matrix(2,4)
    rotated_point.z = result_matrix(3,4)


if __name__ == '__main__':
    run()
