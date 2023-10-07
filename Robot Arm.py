import numpy
import matplotlib.pyplot as plt

def run():
    length1  = 62
    length2  = 93
    length3  = 93
    length4  = 62
    length5  = 93
    length6  = 61
    initialTheta = -90
    finalTheta   =  90

def xRotation(point, theta):
    cosTheta = numpy.cos(theta)
    sinTheta = numpy.sin(theta)
    matrixA = point.matrix
    resultMatrix = matrixA

    resultMatrix[0,1] =  matrixA[0,1]*cosTheta + matrixA[0,2]*sinTheta
    resultMatrix[0,2] = -matrixA[0,1]*sinTheta + matrixA[0,2]*cosTheta
    resultMatrix[1,1] =  matrixA[1,1]*cosTheta + matrixA[1,2]*sinTheta
    resultMatrix[1,2] = -matrixA[1,1]*sinTheta + matrixA[1,2]*cosTheta
    resultMatrix[2,1] =  matrixA[2,1]*cosTheta + matrixA[2,2]*sinTheta
    resultMatrix[2,2] = -matrixA[2,1]*sinTheta + matrixA[2,2]*cosTheta
    resultMatrix[3,1] =  matrixA[3,1]*cosTheta + matrixA[3,2]*sinTheta
    resultMatrix[3,2] = -matrixA[3,1]*sinTheta + matrixA[3,2]*cosTheta

    rotatedPoint.matrix = resultMatrix
    rotatedPoint.x = resultMatrix(1,4)
    rotatedPoint.y = resultMatrix(2,4)
    rotatedPoint.z = resultMatrix(3,4)


if __name__ == '__main__':
    run()
