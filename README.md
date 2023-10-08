# Robot_Visualization

This is the explication of how to draft a 6 DOF Robot Arm

To start we define the initial point $x=0$, $y=0$, $z=0$

To start making the calculous we need to define the caracteristics of the joints in the robot arm, for this we are going to use servos RDS51150, to simplify the draft we use:

$length_1  = 62$  
$length_2  = 93$  
$length_3  = 93$  
$length_4  = 62$  
$length_5  = 93$  
$length_6  = 61$  

The initial point of the first servo is:  

$$
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$
To calculate the final position of servo_1 we need to translate the matrix in z
