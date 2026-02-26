# <center>6. Appendix : Python Interface & CLI Example</center>







<br/>

## Table of Contents
1. Configuration of a Python Interface (common2/imp/DSR_ROBOT2.py)
2. Example Function (CLI Commands)






<br/>

## 1) Configuration of a Python Interface (common2/imp/DSR_ROBOT2.py)

In ROS2, service communication is used to send operation commands to a robot.<br/>
This service communication, implemented as a Python function, is called a Python interface.<br/>
The Python interface has a form similar to DRL (Doosan Robot Language).<br/>


<br/>

---
![alt text](../image/6_1_DSR_ROBOT2.png)
<center>Create Client</center>
<center>(common2/imp/DR_ROBOT2.py)</center>

<br/><br/>

---
![alt text](../image/6_2_DSR_ROBOT2.png)
<center>Motion Function Definition</center>
<center>(common2/imp/DR_ROBOT2.py)</center>

<br/><br/>

---
![alt text](../image/6_3_DSR_ROBOT2.png)
<center>Service Request within a Function</center>
<center>(common2/imp/DR_ROBOT2.py)</center>

<br/><br/><br/><br/>




Examples of some functions in the Python interface can be found in the file at the path below.<br/>
1.  doosan-robot2/dsr_example2/example/example/simple/single_robot_simple.py
2.  doosan-robot2/edu_example/edu_example/extra/test_function.py

<br/><br/><br/><br/>

## 2) Example Function (CLI Commands)


The format of the argument for the command can be found in the paths "doosan-robot2/msg" and "doosan-robot2/srv".  

<br/>

### JogMultiAxis (Topic)
```bash
ros2 topic pub --once /dsr01/jog_multi dsr_msgs2/msg/JogMultiAxis "{jog_axis: [1,0,0,0,0,0], move_reference: 0, speed: 20}"
ros2 topic pub --once /dsr01/jog_multi dsr_msgs2/msg/JogMultiAxis "{jog_axis: [-1,0,0,0,0,0], move_reference: 0, speed: 20}"
ros2 topic pub --once /dsr01/jog_multi dsr_msgs2/msg/JogMultiAxis "{jog_axis: [0,0,0,0,0,0], move_reference: 0, speed: 20}"

# jog_axis       : unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
# move_reference : 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
# speed          : jog speed [%]
```

<br/>

### Kinematics
```bash
# Forward kinematics
ros2 service call /dsr01/motion/fkin dsr_msgs2/srv/Fkin "{pos: [0,0,90,0,90,0]}" # j -> x

# Inverse kinematics
ros2 service call /dsr01/motion/ikin dsr_msgs2/srv/Ikin "{pos: [300,0,300,0,180,0], sol_space: 2}" # x-> j

# Forward kinematics is the field of mechanics where the necessary inputs are calculated to control the position of a system.
# Inverse kinematics is the field of mechanics where the position of a system is calculated based on given inputs.
```

<br/>

### MoveJoint
```bash
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint   "{pos: [   0,   0,  90,   0,  90,   -30], vel: 30, acc: 30}"

# pos                      # target joint angle list [degree] 
# vel                      # set velocity: [deg/sec]
# acc                      # set acceleration: [deg/sec2]
# time (defualt value : 0) # Time [sec] 
# mode (defualt value : 0) # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
```

<br/>

### MoveLine
```bash
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [400,0,400,0,180,0], vel: [200, 200], acc: [200, 200]}"

# pos                      # target  
# vel                      # set velocity: [mm/sec], [deg/sec]
# acc                      # set acceleration: [mm/sec2], [deg/sec2]
# time (defualt value : 0) # Time [sec] 
# ref                      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
# mode (defualt value : 0) # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
```

<br/>

### MoveCircle
```bash
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,0,500,0,180,0], vel: 200, acc: 200, sol: 2}" # start position
ros2 service call /dsr01/motion/move_circle dsr_msgs2/srv/MoveCircle "{pos: [
    {data: [300, 100,500,0,180,0]}, 
    {data: [300,-100,500,0,180,0]}, 
    ], vel: [200, 200], acc: [200, 200]}"

# pos                       # target positions [2][6]  
# vel                       # set velocity: [mm/sec], [deg/sec]
# acc                       # set acceleration: [mm/sec2], [deg/sec2]
```

<br/>

### MoveSplineJoint
```bash
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [  0,0,90,0,90,0], vel: 100, acc: 100}"
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [ 45,0,90,0,90,0], vel: 100, acc: 100}"
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [22.5,0,45,0,0,0], vel: 100, acc: 100}"
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [  0,0,90,0,90,0], vel: 100, acc: 100}"
ros2 service call /dsr01/motion/move_spline_joint dsr_msgs2/srv/MoveSplineJoint "{pos: [
    {data: [  0,0,90,0,90,0]}, 
    {data: [ 45,0,90,0,90,0]}, 
    {data: [22.5,0,45,0,0,0]}, 
    {data: [  0,0,90,0,90,0]}, 
    ], pos_cnt: 4, time: 5,
    vel: [100,100,100,100,100,100],
    acc: [100,100,100,100,100,100]}"    

# pos                      # target positions [100][6] (max num : 100)
# pos_cnt                  # target cnt 
# vel                      # set joint velocity: [deg/sec]
# acc                      # set joint acceleration: [deg/sec2] 
# time (defualt value : 0) # Time [sec] 
# mode (defualt value : 0) # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
```

<br/>

### MoveSplineTask
```bash
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [400,200,400,0,180,0], vel: [200, 200], acc: [200, 200]}"
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [200,200,400,0,180,0], vel: [200, 200], acc: [200, 200]}"
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [200,  0,400,0,180,0], vel: [200, 200], acc: [200, 200]}"
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine "{pos: [400,  0,400,0,180,0], vel: [200, 200], acc: [200, 200]}"
ros2 service call /dsr01/motion/move_spline_task dsr_msgs2/srv/MoveSplineTask "{pos: [
    {data: [400,200,400,0,180,0]}, 
    {data: [200,200,400,0,180,0]}, 
    {data: [200,  0,400,0,180,0]}, 
    {data: [400,  0,400,0,180,0]}, 
    ], pos_cnt: 4, time: 5, vel: [200, 200], acc: [200, 200]}"

# pos                      # target positions [100][6] (max num : 100)
# pos_cnt                  # target cnt 
# vel                      # set velocity: [mm/sec], [deg/sec]
# acc                      # set acceleration: [mm/sec2], [deg/sec2]
# time (defualt value : 0) # Time [sec] 
# ref                      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
# mode (defualt value : 0) # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
```

<br/>

### MoveBlending
```bash
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,  0,400,0,180,0], vel: 100, acc: 100, sol: 2}" 
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,200,400,0,180,0], vel: 100, acc: 100, sol: 2}" 
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [300,200,400,0,180,0], vel: 100, acc: 100, sol: 2}" 
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [200,200,400,0,180,0], vel: 100, acc: 100, sol: 2}" # circle 1
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [200,100,400,0,180,0], vel: 100, acc: 100, sol: 2}" # circle 2
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [200,  0,400,0,180,0], vel: 100, acc: 100, sol: 2}" 
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,  0,400,0,180,0], vel: 100, acc: 100, sol: 2}" # start position
ros2 service call /dsr01/motion/move_blending dsr_msgs2/srv/MoveBlending "{segment: [
    {data: [400,200,400,0,180,0,   0,  0,  0,  0,  0,  0,  0,20]},
    {data: [300,200,400,0,180,0,   0,  0,  0,  0,  0,  0,  0,20]},
    {data: [200,200,400,0,180,0, 200,100,400,  0,180,  0,  1,20]}, # circle
    {data: [200,  0,400,0,180,0,   0,  0,  0,  0,  0,  0,  0,20]},
    {data: [400,  0,400,0,180,0,   0,  0,  0,  0,  0,  0,  0,20]},
    ], pos_cnt: 5, vel: [100, 100], acc: [100, 100]}"
```

<br/>

### MovePeriodic
```bash
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,0,300,0,180,0], vel: 200, acc: 200, sol: 2}"
ros2 service call /dsr01/motion/move_periodic dsr_msgs2/srv/MovePeriodic "{amp: [30,0,0,0,0,0], periodic: [1,1,1,1,1,1], acc: 0.2, repeat: 1, ref: 0}"
ros2 service call /dsr01/motion/move_periodic dsr_msgs2/srv/MovePeriodic "{amp: [0,0,0,0,0,30], periodic: [1,1,1,1,1,1], acc: 0.2, repeat: 1, ref: 0}"

# amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   
# periodic         # Period (time for 1 cycle) [sec]
# acc              # Acc-, dec- time [sec] 
# repeat           # Repetition count 
# ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
```

<br/>

### Jog
```bash
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 0}"
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,0,300,0,180,0], vel: 200, acc: 200, sol: 2}"
ros2 service call /dsr01/motion/jog dsr_msgs2/srv/Jog "{jog_axis: 1, move_reference: 0, speed: -20}"
ros2 service call /dsr01/motion/jog dsr_msgs2/srv/Jog "{jog_axis: 1, move_reference: 0, speed: 0}"
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}"

# jog_axis        # 0 ~ 5 : JOINT 1 ~ 6 
#                 # 6 ~ 11: TASK 1 ~ 6 (X,Y,Z,rx,ry,rz)
# move_reference  # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL
# speed           # jog speed [%] : + forward , 0=stop, - backward  
```

<br/>

### JogMulti
```bash
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,0,300,0,180,0], vel: 200, acc: 200, sol: 2}"
ros2 service call /dsr01/motion/jog_multi dsr_msgs2/srv/JogMulti "{jog_axis: [1,1,0,0,0,0], move_reference: 0, speed: 20}"
ros2 service call /dsr01/motion/jog_multi dsr_msgs2/srv/JogMulti "{jog_axis: [0,0,0,0,0,0], move_reference: 0, speed: 20}"
ros2 service call /dsr01/motion/move_jointx dsr_msgs2/srv/MoveJointx "{pos: [400,0,300,0,180,0], vel: 200, acc: 200, sol: 2}"

# jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
# move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
# speed             # jog speed [%]  
```
