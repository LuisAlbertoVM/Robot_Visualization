# Robot_Visualization

This is the explication of how to draft a 6 DOF Robot Arm

To start making the calculous we need to define the caracteristics of the joints in the robot arm, for this we are going to use servos RDS51150, to simplify the draft we use:

$length_1  = 62$  
$length_2  = 93$  
$length_3  = 93$  
$length_4  = 62$  
$length_5  = 93$  
$length_6  = 61$  

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
a_0_0 & a_01 & a_02 & a_03 \\
a_10 & a_11 & a_12 & a_13 \\
a_20 & a_21 & a_22 & a_23 \\
a_30 & a_31 & a_32 & a_33
\end{bmatrix}
x
\begin{bmatrix}
b_00 & b_01 & b_02 & b_03 \\
b_10 & b_11 & b_12 & b_13 \\
b_20 & b_21 & b_22 & b_23 \\
b_30 & b_31 & b_32 & b_33
\end{bmatrix}
$$
$$
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

To start we define the initial point $x=0$, $y=0$, $z=0$ and the initial point of the first servo is:

$$
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$