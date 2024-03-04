# Robot_Visualization

This is the explication of how to draft a 6 DOF Robot Arm

To start making the calculous we need to define the caracteristics of the joints in the robot arm, for this we are going to use servos RDS51150, to simplify the draft we use:

$$length_1  = 62$$
  
$$length_2  = 93$$  

$$length_3  = 93$$  

$$length_4  = 62$$  

$$length_5  = 93$$  

$$length_6  = 61$$  

```python
length1  = 62
length2  = 93
length3  = 93
length4  = 62
length5  = 93
length6  = 61
```

The translations and rotations matrix are:

$$
translation_x = 
\begin{bmatrix}
1 & 0 & 0 & x \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
translation_y = 
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
translation_z = 
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
rotation_x = 
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & cos(\theta) & -sin(\theta) & 0 \\
0 & sin(\theta) & cos(\theta) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
rotation_y = 
\begin{bmatrix}
cos(\theta) & 0 & sin(\theta) & 0 \\
0 & 1 & 0 & 0 \\
-sin(\theta) & 0 & cos(\theta) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
rotation_z = 
\begin{bmatrix}
cos(\theta) & -sin(\theta) & 0 & 0 \\
sin(\theta) & cos(\theta) & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Observe that the we can improve the matrix multiplication making this multiplication:

$$
\begin{bmatrix}
a_{00} & a_{01} & a_{02} & a_{03} \\
a_{10} & a_{11} & a_{12} & a_{13} \\
a_{20} & a_{21} & a_{22} & a_{23} \\
a_{30} & a_{31} & a_{32} & a_{33}
\end{bmatrix}
x
\begin{bmatrix}
b_{00} & b_{01} & b_{02} & b_{03} \\
b_{10} & b_{11} & b_{12} & b_{13} \\
b_{20} & b_{21} & b_{22} & b_{23} \\
b_{30} & b_{31} & b_{32} & b_{33}
\end{bmatrix}
->
\begin{bmatrix}
c_{00} & c_{01} & c_{02} & c_{03} \\
c_{10} & c_{11} & c_{12} & c_{13} \\
c_{20} & c_{21} & c_{22} & c_{23} \\
c_{30} & c_{31} & c_{32} & c_{33}
\end{bmatrix}
$$

Where $C$ is:

$$c_{00} = a_{00} b_{00} + a_{01} b_{10} + a_{02} b_{20} + a_{03} b_{30}$$

$$c_{01} = a_{00} b_{01} + a_{01} b_{11} + a_{02} b_{21} + a_{03} b_{31}$$

$$c_{02} = a_{00} b_{02} + a_{01} b_{12} + a_{02} b_{22} + a_{03} b_{32}$$

$$c_{03} = a_{00} b_{03} + a_{01} b_{13} + a_{02} b_{23} + a_{03} b_{33}$$

$$c_{10} = a_{10} b_{00} + a_{11} b_{10} + a_{12} b_{20} + a_{13} b_{30}$$

$$c_{11} = a_{10} b_{01} + a_{11} b_{11} + a_{12} b_{21} + a_{13} b_{31}$$

$$c_{12} = a_{10} b_{02} + a_{11} b_{12} + a_{12} b_{22} + a_{13} b_{32}$$

$$c_{13} = a_{10} b_{03} + a_{11} b_{13} + a_{12} b_{23} + a_{13} b_{33}$$

$$c_{20} = a_{20} b_{00} + a_{21} b_{10} + a_{22} b_{20} + a_{23} b_{30}$$

$$c_{21} = a_{20} b_{01} + a_{21} b_{11} + a_{22} b_{21} + a_{23} b_{31}$$

$$c_{22} = a_{20} b_{02} + a_{21} b_{12} + a_{22} b_{22} + a_{23} b_{32}$$

$$c_{23} = a_{20} b_{03} + a_{21} b_{13} + a_{22} b_{23} + a_{23} b_{33}$$

$$c_{30} = a_{30} b_{00} + a_{31} b_{10} + a_{32} b_{20} + a_{33} b_{30}$$

$$c_{31} = a_{30} b_{01} + a_{31} b_{11} + a_{32} b_{21} + a_{33} b_{31}$$

$$c_{32} = a_{30} b_{02} + a_{31} b_{12} + a_{32} b_{22} + a_{33} b_{32}$$

$$c_{33} = a_{30} b_{03} + a_{31} b_{13} + a_{32} b_{23} + a_{33} b_{33}$$

As this is the solution of a multiplication of 2 4x4 Matrix, we save time not procesing the comon way to multiplicate 2 matrix that has 3 for

The solution of translate a matrix A in x is:

$$C = A$$

$$C[:,3] = A[:,3] + A[:,0]*x$$

```python
def translation_x(point, x):
    result_matrix = np.copy(point.matrix)
    result_matrix[:, 3] += result_matrix[:, 0] * x
    return Point(matrix=result_matrix)
```

The solution of translate a matrix A in y is:

$$C = A$$

$$C[:,3] = A[:,3] + A[:,1]*y$$

```python
def translation_y(point, y):
    result_matrix = np.copy(point.matrix)
    result_matrix[:, 3] += result_matrix[:, 1] * y
    return Point(matrix=result_matrix)
```

The solution of translate a matrix A in z is:

$$C = A$$

$$C[:,3] = A[:,3] + A[:,2]*z$$

```python
def translation_z(point, z):
    result_matrix = np.copy(point.matrix)
    result_matrix[:, 3] += result_matrix[:, 2] * z
    return Point(matrix=result_matrix)
```

The rotation functions are:

```python
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
```

To start we define the initial point $x=0$, $y=0$, $z=0$ and the initial point of the first servo is:

```python
class Point:
    def __init__(self, matrix):
        self.matrix = matrix
        self.x = self.matrix[0][3]
        self.y = self.matrix[1][3]
        self.z = self.matrix[2][3]
```

$$
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

```python
initial_matrix_servo_1 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
```

The initial and final positions of the servos are:

$$Servo1_{Final Position} = Servo1_{Initial Position} * translation_z(length_1) * rotation_z(\theta_1)$$

$$Servo2_{Initial Position} = Servo1_{Final Position}$$

$$Servo2_{Final Position} = Servo2_{InitialPosition} * rotation_y(\theta_2) * translation_z(length_2)$$

$$Servo3_{Initial Position} = Servo2_{Final Position}$$

$$Servo3_{Final Position} = Servo3_{InitialPosition} * rotation_y(\theta_3) * translation_z(length_3)$$

$$Servo4_{Initial Position} = Servo3_{Final Position}$$

$$Servo4_{Final Position} = Servo4_{InitialPosition} * rotation_z(\theta_4) * translation_z(length_4)$$

$$Servo5_{Initial Position} = Servo4_{Final Position}$$

$$Servo5_{Final Position} = Servo5_{InitialPosition} * rotation_y(\theta_5) * translation_z(length_5)$$

$$Servo6_{Initial Position} = Servo5_{Final Position}$$

$$Servo6_{Final Position} = Servo6_{InitialPosition} * rotation_z(\theta_6) * translation_z(length_6)$$

```python
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
```

To check that a servo does not collide with another, an approximation check that no corner of the servo is inside another servo, so no corner of the servo will be between the 6 sides of the other servos, for this we create a plane for each of the faces and calculate the distance from each corner of the servo to each of the planes resulting in that the distance between the servo corner is greater than the height, width and length of the servo in any of the 6 planes.

$$vector_1 = point_2 - point_1$$

$$vector_2 = point_3 - point_1$$

$$normal = vector_1 * vector 2$$

$$plane = Ax + By + Cz + D = 0$$

$$A = normal[0]$$

$$B = normal[1]$$

$$C = normal[2]$$

$$D = -normal . point_1$$

```python
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
```

To calculate de distance from one point to a plane:

$$Distance = \frac{a \cdot x + b \cdot y + c \cdot z + d}{\sqrt{a^2 + b^2 + c^2}}$$

```python
def distance_from_point_to_plane(point, plane):
    return abs(plane.a*point.x + plane.b*point.y + plane.c*point.z + plane.d)/math.sqrt(plane.a ** 2 + plane.b ** 2 + plane.c ** 2)
```

To know if a point is inside a servo:

```python
def translate_point_in_x_y_z(initial_point, x, y, z):
    translation_z(translation_y(translation_x(initial_point,x),y),z)

```


```python
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
        
        self.point_1 = translate_point_in_x_y_z(initial_point,            -(15/65) * self.width, self.depth/2, 7)
        self.point_2 = translate_point_in_x_y_z(initial_point,  self.width-(15/65) * self.width, self.depth/2, 7)
        self.point_3 = translate_point_in_x_y_z(initial_point, -self.width-(15/65) * self.width, self.depth/2, 7)
        self.point_4 = translate_point_in_x_y_z(initial_point,            -(15/65) * self.width, self.depth/2, 7)
        self.point_5 = translate_point_in_x_y_z(initial_point,            -(15/65) * self.width, self.depth/2, self.heigth+7)
        self.point_6 = translate_point_in_x_y_z(initial_point,  self.width-(15/65) * self.width, self.depth/2, self.heigth+7)
        self.point_7 = translate_point_in_x_y_z(initial_point, -self.width-(15/65) * self.width, self.depth/2, self.heigth+7)
        self.point_8 = translate_point_in_x_y_z(initial_point,            -(15/65) * self.width, self.depth/2, self.heigth+7)
        
        self.plane_down     = Plane(self.point_1, self.point_2, self.point_4)
        self.plane_up       = Plane(self.point_5, self.point_6, self.point_8)
        self.plane_forward  = Plane(self.point_1, self.point_2, self.point_5)
        self.plane_backward = Plane(self.point_4, self.point_3, self.point_8)
        self.plane_left     = Plane(self.point_1, self.point_5, self.point_4)
        self.plane_rigth    = Plane(self.point_3, self.point_2, self.point_7)
```

```python
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
```

```python
def is_two_servo_colision(servo_1, servo_2):
    is_point_1_inside_servo_2 = is_point_inside_servo(servo_1.point_1, servo_2)
    is_point_2_inside_servo_2 = is_point_inside_servo(servo_1.point_2, servo_2)
    is_point_3_inside_servo_2 = is_point_inside_servo(servo_1.point_3, servo_2)
    is_point_4_inside_servo_2 = is_point_inside_servo(servo_1.point_4, servo_2)
    is_point_5_inside_servo_2 = is_point_inside_servo(servo_1.point_5, servo_2)
    is_point_6_inside_servo_2 = is_point_inside_servo(servo_1.point_6, servo_2)
    is_point_7_inside_servo_2 = is_point_inside_servo(servo_1.point_7, servo_2)
    is_point_8_inside_servo_2 = is_point_inside_servo(servo_1.point_8, servo_2)

    return is_point_1_inside_servo_2 or is_point_2_inside_servo_2 or is_point_3_inside_servo_2 or is_point_4_inside_servo_2 or is_point_5_inside_servo_2 or is_point_6_inside_servo_2 or is_point_7_inside_servo_2 or is_point_8_inside_servo_2
```
