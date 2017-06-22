## Udacity Robotics Nanodegree Project 2

# Robotic Arm: Pick and Place

In this assignment, the task is to control a Kuka KR210 simulator to pick a target from the shelf and drop it in the bin.

The path is decided by the simulator, and is random for every run. The challenge is to follow the given path as close as possible, and complete the operation.

There are 9 different spawn location of the target object in the shelf.


### Kinematics Analysis 1: DH Parameters for Kuka KR210

![Figure 1](https://github.com/ongchinkiat/robond-pick-and-place/raw/master/figure1.jpg "Figure 1")



The DH parameters are determined with the help of the diagram and the KR210 URDF file.

| Parameter   |  Explanation  | Value |
| -----       | -----         | ----- |
| alpha<sub>0</sub> | Z<sub>0</sub> Parallel to Z<sub>1</sub>        |  0 |
| alpha<sub>1</sub> | Z<sub>1</sub> Perpendicular to Z<sub>2</sub>        |  -90&deg; |
| alpha<sub>2</sub> | Z<sub>2</sub> Parallel to Z<sub>3</sub>        |  0 |
| alpha<sub>3</sub> | Z<sub>3</sub> Perpendicular to Z<sub>4</sub>        |  -90&deg; |
| alpha<sub>4</sub> | Z<sub>4</sub> Perpendicular to Z<sub>5</sub>        |  90&deg; |
| alpha<sub>5</sub> | Z<sub>5</sub> Perpendicular to Z<sub>6</sub>        |  -90&deg; |
| alpha<sub>6</sub> | Z<sub>6</sub> Parallel to Z<sub>7</sub>        |  0 |
| a<sub>0</sub>     | O<sub>1</sub> - O<sub>0</sub> along X<sub>1</sub>        | 0 |
| a<sub>1</sub>     | O<sub>2</sub> - O<sub>1</sub> along X<sub>2</sub>        | 0.75 |
| a<sub>2</sub>     | O<sub>3</sub> - O<sub>2</sub> along X<sub>3</sub>        | 2 - 0.75 = 1.25 |
| a<sub>3</sub>     | O<sub>4</sub> - O<sub>3</sub> along X<sub>4</sub>        | 1.946 - 2 = -0.054 |
| a<sub>4</sub>     | O<sub>5</sub> - O<sub>4</sub> along X<sub>5</sub>        | 0 |
| a<sub>5</sub>     | O<sub>6</sub> - O<sub>5</sub> along X<sub>6</sub>        | 0 |
| a<sub>6</sub>     | O<sub>7</sub> - O<sub>6</sub> along X<sub>7</sub>        | 0 |
| d<sub>1</sub>     | O<sub>1</sub> - O<sub>0</sub> along Z<sub>1</sub>        | 0.33 |
| d<sub>2</sub>     | O<sub>2</sub> - O<sub>1</sub> along Z<sub>2</sub>        | 0 |
| d<sub>3</sub>     | O<sub>3</sub> - O<sub>2</sub> along Z<sub>3</sub>        | 0 |
| d<sub>4</sub>     | O<sub>4</sub> - O<sub>3</sub> along Z<sub>4</sub>        | 1.85 - 0.35 = 1.50 |
| d<sub>5</sub>     | O<sub>5</sub> - O<sub>4</sub> along Z<sub>5</sub>        | 0 |
| d<sub>6</sub>     | O<sub>6</sub> - O<sub>5</sub> along Z<sub>6</sub>        | 0 |
| d<sub>7</sub>     | O<sub>7</sub> - O<sub>6</sub> along Z<sub>7</sub>        | 2.153 - 1.85 = 0.303 |
| theta<sub>1</sub> | X<sub>0</sub> Parallel to X<sub>1</sub>        |  theta<sub>1</sub> |
| theta<sub>2</sub> | X<sub>1</sub> Perpendicular to X<sub>2</sub>        |  theta<sub>2</sub> - 90&deg; |
| theta<sub>3</sub> | X<sub>2</sub> Parallel to X<sub>3</sub>        |  theta<sub>3</sub> |
| theta<sub>4</sub> | X<sub>3</sub> Parallel to X<sub>4</sub>        |  theta<sub>4</sub> |
| theta<sub>5</sub> | X<sub>4</sub> Parallel to X<sub>5</sub>        |  theta<sub>5</sub> |
| theta<sub>6</sub> | X<sub>5</sub> Parallel to X<sub>6</sub>        |  theta<sub>6</sub> |
| theta<sub>7</sub> | X<sub>6</sub> Parallel to X<sub>7</sub>        |  theta<sub>7</sub> |




The DH parameter table



| i     | alpha<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub>   | theta<sub>i</sub> |
| ----- | -----     | ----- | ----- | -----   |
| 1     | 0         |   0.35|  0.75 |       theta<sub>1</sub> |
| 2     | -90&deg;       |   1.25|     0 |       theta<sub>2</sub> - 90&deg; |
| 3     | 0         | -0.054|   1.5 |       theta<sub>3</sub> |
| 4     | -90&deg;       |     0 |     0 |       theta<sub>4</sub> |
| 5     | 90&deg;        |     0 |     0 |       theta<sub>5</sub> |
| 6     | -90&deg;       |     0 |     0 |       theta<sub>6</sub> |
| 7     | 0         |     0 | 0.303 |       theta<sub>7</sub> |

### Kinematics Analysis 2: Transformation matrices using DH Parameters

Given the current joint and joint angle theta<sub>i</sub>, we can determine the co-ordinates of the next joint by using the Transformation matrix.

The transformation matrix is a combination of 4 operations (2 rotations and 2 translations) using the 4 DH parameters alpha<sub>i-1</sub>, a<sub>i-1</sub>, d<sub>i</sub>, theta<sub>i</sub>.

1. Rotation along X axis by alpha<sub>i-1</sub>
2. Translation along X axis by a<sub>i-1</sub>
3. Rotation along Z axis by theta<sub>i</sub>
4. Translation along Z axis by d<sub>i-1</sub>

![Figure 2](https://github.com/ongchinkiat/robond-pick-and-place/raw/master/figure3.jpg "Figure 2")


### Kinematics Analysis 3: Inverse Kinematics

In Inverse Kinematics, we are given the End-effector Position Px, Py, Pz and Orientation, and we need to find the joint angles to reach this target.

We can break down the Inverse Kinematics into 2 parts: Position and Orientation.

#### Position

First, we need to get the location of the wrist center W =  O<sub>5</sub>.

Since W is located from P travel along the orientation angle (n) over distance (d<sub>6</sub> + End-effector length l), W can be determined by these equations:

1. W<sub>x</sub> = P<sub>x</sub> - (d<sub>6</sub> + l) * n<sub>x</sub>
2. W<sub>y</sub> = P<sub>y</sub> - (d<sub>6</sub> + l) * n<sub>y</sub>
3. W<sub>z</sub> = P<sub>z</sub> - (d<sub>6</sub> + l) * n<sub>z</sub>


Joint 1 angle theta<sub>1</sub> is determined by projecting W onto the ground plane.

theta<sub>1</sub> = atan2(W<sub>y</sub>, W<sub>x</sub>)

Using theta<sub>1</sub>, we can determine the location of Origin 2 O<sub>2</sub>.

Joint 2 and 3 angle theta<sub>2</sub> and theta<sub>3</sub> can be determined by visualising using the following diagram:

![Figure 3](https://github.com/ongchinkiat/robond-pick-and-place/raw/master/figure2.jpg "Figure 3")

The length of all 3 sides of the triangles are known:
1. O<sub>2</sub>O<sub>3</sub> = a<sub>2</sub> = 1.25
2. O<sub>3</sub>W = sqrt( (1.85-0.35)<sup>2</sup>  + (1.946-2.0)<sup>2</sup> )
3. O<sub>2</sub>W = sqrt( (W<sub>x</sub> - O<sub>2x</sub>)<sup>2</sup> +  (W<sub>y</sub> - O<sub>2y</sub>)<sup>2</sup> + (W<sub>z</sub> - O<sub>2z</sub>)<sup>2</sup>)

The drop along Z axis from O<sub>2</sub> to W

O<sub>2z</sub>W = O<sub>2z</sub> - W<sub>z</sub>

Find angle g:

sin(g) = O<sub>2z</sub>W / O<sub>2</sub>W

Find angle f:

cos(f) = ( (O<sub>2</sub>O<sub>3</sub>)<sup>2</sup> + (O<sub>2</sub>W)<sup>2</sup> - (O<sub>3</sub>W)<sup>2</sup> ) / (2 \* O<sub>2</sub>O<sub>3</sub> \* O<sub>2</sub>W)

Then we can get theta<sub>2</sub>

theta<sub>2</sub> = 90&deg; + g - f

With theta<sub>2</sub>, we can get the location of O<sub>3</sub>.

The drop along Z axis from O<sub>3</sub> to W
O<sub>3z</sub>W = O<sub>3z</sub> - W<sub>z</sub>

Finally, get theta<sub>3</sub>

sin(theta<sub>3</sub> + theta<sub>2</sub>) = O<sub>3z</sub>W / O<sub>3</sub>W

#### Orientation

By feeding the calculated theta<sub>1</sub>, theta<sub>2</sub>, theta<sub>3</sub> into the Transformation matrix, we can get the orientation of the arm at the theta<sub>4</sub>, theta<sub>5</sub>, theta<sub>6</sub> = 0 configuration.

By subtracting this orientation values with the target pose orientation, we get the required rotation needed in the X-Y-Z axis.

But since Joint 4,5,6 are not in the X-Y-Z orientation, converting them to theta<sub>4</sub>, theta<sub>5</sub>, theta<sub>6</sub> is not trival.

Instead, an initial value can be estimated, and further refined using gradient descent numerical method.

### Project Implementation

The equations are implemented in the IK_server.py file to control the pick and place operation.

The generation of the Transformation Matrices and the calculation of the transforms are very slow on my PC, so I made a few adjustments for performance purpose
1. Generation of the transformation matrices generate_forward() is only done once, and the results are saved in pickle files.
2. The Forward Kinematics calculator k_cal() tries to load the matrices from the pickle file, if fail it runs the generate_forward() function to generate a fresh set of files.
3. The orientation angles (theta4, theta5, theta6) are only calculated in the last pose for the Pick phase, since calculating them need at least 1 run of the forward kinematics calculation.
4. The positional angles (theta1, theta2, theta3) are calculated in ik_cal_xyz() base on the end-effector position, instead of the wrist center position, since we are setting (theta4, theta5, theta6) = 0 most of the time.

The steps for each given pose:
1. calculate IK for joint 1,2,3 (theta1, theta2, theta3) using simple geometry
```
theta1, theta2, theta3 = ik_cal_xyz(px,py,pz)
```

2. get the orientation of spherical wrist given (theta1, theta2, theta3), setting (theta4, theta5, theta6) = 0
```
x1, y1, z1, Inv_R0_4_c, final = k_cal(theta1, theta2, theta3, theta4, theta5, theta6)
fquad = quad_from_matrix(final)
(ct4, ct5, ct6) = tf.transformations.euler_from_quaternion(fquad)
```

3. get the target orientation given by the target pose
```
(target_euler4, target_euler5, target_euler6) = tf.transformations.euler_from_quaternion(target_quad)
```

4. find the rotation we need to move the wrist to the target orientation
```
dt4 = ct4 - target_euler4
dt5 = ct5 - target_euler5
dt6 = ct6 - target_euler6
```

5. move joints 4,5,6 towards the target orientation
```
theta4 -= dt4
# since joints 4,5,6 is in the Z-X-Z orientation, we need some complex calculation to
# find a solution to match all 3 target rotation
# for simplicity, we adjust to only the larger angle of dt5 or dt6
if (abs(dt5) > abs(dt6)):
    theta5 -= dt5
else:
    # turn theta4 90 degrees and theta6 -90 degress so that theta5 rotates along Z axis
    theta4 += pi/2
    theta6 -= pi/2
    theta5 -= dt6
```

6. after rotation, the End-Effector position is changed. We do translation adjust again by using forward kinematics to find the current x,y,z and try to adjust it back to the given px, py, pz
```
x1, y1, z1, Inv_R0_4_c, final = k_cal(theta1, theta2, theta3, theta4, theta5, theta6)
dx = px - x1
dy = py - y1
dz = pz - z1
adjust_x = px + dx
adjust_y = py + dy
adjust_z = pz + dz
theta1, theta2, theta3 = ik_cal_xyz(adjust_x,adjust_y,adjust_z)
```

7. For more accurate result, we can keep repeating Step 2 to 6

This implementation compromise accuracy for speed.

In the simulation runs, the final orientation of the gripper is good enough for the arm to grip the target consistently, without missing or knocking off the target.

I have uploaded a video of the simulator running this code to pick up the target at all 9 positions on the shelf.

Video URL: https://youtu.be/TOrrYUh3He4

<a href="http://www.youtube.com/watch?feature=player_embedded&v=TOrrYUh3He4" target="_blank"><img src="http://img.youtube.com/vi/TOrrYUh3He4/0.jpg"
alt="Pick and place" width="240" height="180" border="1" /></a>
