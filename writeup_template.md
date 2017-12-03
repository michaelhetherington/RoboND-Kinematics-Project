## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/fk.png
[image3]: ./misc_images/theta1.jpg
[image4]: ./misc_images/theta2.jpg
[image5]: ./misc_images/theta3.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis

The kinematic analysis entails three stages once the position of the target object and drop-bin are known (these target locations are provided by Gazebo):

0. Initiate Gazebo and RViz demo of Kura KR210

   Using the demo mode the Kura KR210 robotics arm the objective of the project can be visualised and tested.

1. Develop DH paramter matrix

   The DH parameter matrix is used to store the variables that allow us to construct the transformation matrices for the specific robotic arm being used - the Kura KR210 in this project. Without the correct DH parameters it would not be possible to use the transformation matrices to solve either the forward or inverse kinematic problems.

2. Develop transformation matrices from base_link to gripper_link

   These matrices transform the location of the robot base (Joint_0) to the gripper/end-effector (joint 7 named 'Joint_G' in this analysis). This is the solution to the *Forward Kinematics* problem.

3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
   
	 3.1. Inverse Orientation Kinematics
	    
   Develop generic rotation matrices that rotate the gripper/end-effector within the world frame given the *yaw, roll* and *pitch* values provided by Gazebo. These rotational matrices (once extrinsically multiplied) must then by multiplied by a correction matrix to correct for the difference between the world frame and the URDF frame used to specify the Kura KR210 arm configuration.
	 
	 3.2. Invesrse Position Kinematics
	 
   With the rotation of the gripper/end-effector known from the previous step, and it's position within the world frame known as provided by Gazebo we must then calculate the location of the Wrist Centre (WC). Once we're able to calculate the WC we can solve the inverse problem to allow us to position the WC within the world frame. The location of the WC will then allow us to calculate the joint angles for Joint_1, Joint_2 and Joint_3. 
	 
4. Calculate Joint 1, 2 and 3 angles

   With the wrist centre now known from the previous step we can calculate the angles Theta1, Theta2 and Theta3. The calculation of these angles is done using geometry. 

5. Calculate Joint 4, 5 and 6 angles

   Theta4, 5 and 6 are calculated using Euler angle theory as described in the lecture notes.

#### 0. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

This is an image of the Kura KR210 robot arm being simulated in RViz. The joint angle controls are shown on the right hand side in a separate window and enable change of the robot gripper/end-effector pose.

![alt text][image1]

#### 1. First derive the DH parameter table then create individual transformation matrices about each joint.

To schematically describe the robot arm the below figure is adopted. This is provided in the lecture notes for the project and shows the local x and z axes for each joint. It also shows the label for each joint. The lecture notes provided instructions and conventions for generating the schematic. This helps clearly visualise the configuration of each joint.

![alt text][image2]

While the demo can be used to investigate the robot configuration we can also inspect the URDF file that specifies the configuration of each element of the robot arm. An example of the section of the URDF configuration file that describes the configuration of joint 3 in relation to joint 2 is shown below. This example describes that to get from joint_2 to joint_3 a link length of 1.25m is present.

```
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="$$
  </joint>
```

DH parameter table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The transformation matrices from a joint to its successor is generated from a generic transformation matrix:
```python
def Trans_Matrix(alpha, a, d, q):
	MAT = Matrix([[           cos(q),          -sin(q),          0,            a],
	              [sin(q)*cos(alpha),cos(q)*cos(alpha),-sin(alpha),-sin(alpha)*d],
                [sin(q)*sin(alpha),cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                [0,0,0,1]])
                return MAT 
```

By substituing in the symbols from the Modified DH Paramter table (stored in dictionary *s*) the transformation matrices can be constructed one-by-one:
```python
T0_1 = Trans_Matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = Trans_Matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = Trans_Matrix(alpha2, a2, d3, q3).subs(s)
T3_4 = Trans_Matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = Trans_Matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = Trans_Matrix(alpha5, a5, d6, q6).subs(s)
T6_G = Trans_Matrix(alpha6, a6, d7, q7).subs(s)
```
With the transformation matrices from each joint to its successor generated an overall transformation matrix from the base to the gripper/end-effector can be created:
```python
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
This is the result of the Forward Kinematics problem which uses the DH parameters of the Kura KR210 to transform the base link position to the gripper/end-effector position.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### 3.1. Inverse Orientation Kinematics

This step entails computing the rotation of the Kura KR210 gripper given the *yaw, roll* and *pitch* values from Gazebo. 

This is performed by generating generic rotation matrics for each axes and then applying a rotation correction matrix for the difference in the world frame (the chosen reference frame) and the reference frame used in the URDF file that specifies the configuration of the Kura KR210 arm.

The standard rotation matrices are as follows where *r, p* and *y* are the roll, yaw and pitch angles respectively. A conversion from the Gazebo qauternion values to radians must be performed.

```python
            R_z = Matrix([[   cos(y),   -sin(y),         0],
                          [   sin(y),    cos(y),         0],
                          [        0,         0,         1]]) # YAW
    
            R_y = Matrix([[   cos(p),         0,    sin(p)],
                          [        0,         1,         0],
                          [  -sin(p),         0,    cos(p)]]) # PITCH
    
            R_x = Matrix([[        1,         0,         0],
                          [        0,    cos(r),   -sin(r)],
                          [        0,    sin(r),    cos(r)]]) # ROLL
```

The combined rotation matrix is then simply,

```python
R_G = R_z * R_y * R_x
```

However, to account for the difference between the URDF frame and Gazebo (world) frame we must apply a further rotation,

```python
Rot_correction = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
R_G = R_G * Rot_correction
```

Finally we substitute the *roll, pitch* and *yaw* values provided by Gazebo,

```python
R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw})
```

##### 3.2. Inverse Position Kinematics

With the inverse rotation kinematic problem solved we now need to find the location of the WC. This is done using the above result in conjunction with the gripper/end-effector position as provided by Gazebo.

The formula used to scale back to the WC from the gripper/end-effector position is,

```python
WC = POS_G - 0.303 * R_G[:,2]
```

which results in a column matrix that describes the *x, y* and *z* positions of the WC within the Gazebo world. In my code I've created variables WCx, WCy and WCz to store the x, y and z components of the WC respectively.

#### 4. Calculate joint 1, 2 and 3 angles - Theta1, Theta2 and Theta3

The following image shows the general arrangement of Joint 1 to 3 and the WC geometry to calculate Theta1. To visualise and calculate Theta1 the WC position is projected on to the x-y plane.
```python
Theta1 = atan2(WCy, WCx)
```
![alt text][image3]

The next image shows the projection of Joint 2 and 3 on to the y-z plane. Using the cosine rule can solve for A, B and C - the lengths of the triangle made by the two joints and the WC - as well as a_1, b_1 and c_1 - the angles between the sides of the triangle. From this information we can calculate Theta2.
```python
Theta2 = pi/2 - a_1 - atan2(WCz - d1, sqrt(WCx**2 + WCy**2) - a1).subs(s)
```
![alt text][image4]

The next image shows a close-up drawing of Theta3. Calculating this last joint angle was the most challenging and time consuming. For this drawing to work we need to imagine the WC in it's initial position before application of any rotation in Joint 3. Once we do this we apply Theta3 *inside* the right-angle made-up by *b* and *alpha*. This is equivalent to the Theta3 between X2 and X3 axes as provided through the lecture notes. This visualisation provides the correct response.
```python
Theta3 = pi/2 - b - atan2(-a3, d4).subs(s)
```
![alt text][image5]

#### 5. Calculate joint 4, 5 and 6

The joint angles for joints 4, 5 and 6 can be found from the matrix to perform rotations between joint 3 and joint 6. This rotation matrix is extracted from the combined matrix that rotates from the base link to the gripper/end-effector that was created in the above section 3.1. The code to produce this rotation matrix from joint 3 to joint 6 is,

```python
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.inv("LU") * R_G
```

From this extracted rotation matrix we can use Euler angles as described in the lecture notes. The final code for these Theta4, Theta5 and Theta6 joint angles are,

```python
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The key observations I have made are:

  * The gripper rotates *alot* during movement. It isn't clear to me as to why so many rotation iterations are necessary during and in between transformation of the wrist centre. I'd have thought the gripper itself would not need to rotate until the wrist centre was in place
	* I have found on a number of occasions the gripper would knock over the target object when moving in to place because it does not rotate to a horizontal position until after it is at the location it needs to be to grab the target object.
	* The path planning seems as though it does not always take the shortest or most efficient route. This could be caused by inefficiencies in my code that add in additional points along the path
	* When dropping the target object in to the bin it drops it right near the edge. This results in the object actually hitting the edge of the bin and tipping in. With a little bit of induced error I think my robot may not be as successful at placing the object.
	* Unfortunately once again I relied heavily on the walkthrough to understnad the project - particularly the calculation of the joint angles. 

