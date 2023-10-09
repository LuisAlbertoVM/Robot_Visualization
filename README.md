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

$c_{00} = a_{00} b_{00} $

$c_{01} = $

$c_{02} = $

$c_{03} = $

$c_{10} = $

$c_{11} = $

$c_{12} = $

$c_{13} = $

$c_{20} = $

$c_{21} = $

$c_{22} = $

$c_{23} = $

$c_{30} = $

$c_{31} = $

$c_{32} = $

$c_{33} = $

To start we define the initial point $x=0$, $y=0$, $z=0$ and the initial point of the first servo is:

$$
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$