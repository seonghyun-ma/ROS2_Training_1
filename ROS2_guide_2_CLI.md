# <center>2. CLI 명령어 사용 (방법 1)</center>







<br/>

## 목차
1. 서비스 요청 (move_joint)
2. 토픽 발행 (jog_mulit)
3. 토픽 구독 (current_posx)







<br/>

## 초기 설정
작업 공간 이동 및 ROS2 환경 설정을 수행합니다.<br/>
```bash
$ cd ~/ros2_ws # 기본 작업 위치<br/>
$ source /opt/ros/humble/local_setup.bash # ROS 2 환경 설정<br/>
$ source install/setup.bash # 사용자 워크스페이스 환경<br/>
```
<br/>

사전 작업으로, 로봇의 controller 노드를 활성화시킨 다음에 진행합니다.<br/>
```bash
$ ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py
```






<br/><br/>
<!---------------------------------------------------------------->

# 1) 서비스 요청
명령어 인터페이스(CLI, Command Line Interface)는 터미널을 통해 명령을 내리는 방법입니다.<br/>
터미널을 이용하여 서비스를 요청하고 토픽을 구독하는 등, 로봇과 직접 상호 작용할 수 있습니다.<br/>
본 예제에서는 movej 명령을 실행해보겠습니다.<br/>
기본적으로 서비스를 요청하는 CLI 명령어는 다음과 같습니다.<br/>
(서비스 이름, 서비스 형태, 입력 인수)<br/>
``` bash
$ ros2 service call [service_name] [service_type] [arguments]
```







<br/><br/>

## [service_name]
[service_name]에는 요청하고자 하는 서비스의 이름을 입력합니다.<br/>
아래의 명령어로 사용할 수 있는 서비스 목록을 확인할 수 있습니다. <br/>
``` bash
$ ros2 service list

/dsr01/aux_control/get_control_mode
/dsr01/aux_control/get_control_space
/dsr01/aux_control/get_current_posj
...
/dsr01/tool/set_current_tool
/dsr01/tool/set_tool_shape
```


> doosan-robot2/<br/>
> > dsr_msgs2/<br/>
> > > srv/<br/>
> > > > aux_control<br/>
> > > > drl<br/>
> > > > force<br/>
> > > > gripper<br/>
> > > > io<br/>
> > > > modbus<br/>
> > > > motion<br/>
> > > > realtime<br/>
> > > > system<br/>
> > > > tcp<br/>
> > > > tool<br/>
> <br/>


<center>Doosan Robotics ROS2 패키지에서 제공하는 서비스 종류</center>

<br/>

motion과 관련된 서비스만 찾아보는 명령어는 다음과 같습니다.<br/>
이를 통해 movej명령과 관련된 서비스의 이름은 /dsr01/motion/move_joint 인 것을 확인할 수 있습니다.<br/>

``` bash
$ ros2 service list | grep motion

/dsr01/motion/alter_motion
/dsr01/motion/change_operation_speed
/dsr01/motion/check_motion
...
/dsr01/motion/move_joint
/dsr01/motion/move_jointx
/dsr01/motion/move_line
...
/dsr01/motion/set_singularity_handling
/dsr01/motion/trans
```







<br/><br/>

## [service_type]
이제 다음 명령어를 통해 해당 서비스의 형태는 dsr_msgs2/srv/MoveJoint 인 것을 확인할 수 있습니다.

```bash
$ ros2 service type /dsr01/motion/move_joint

dsr_msgs2/srv/MoveJoint
```
<details>
<summary>서비스를 찾는 다른 방법</summary>

다른 방법으로는 아래 명령어를 통해 서비스의 이름과 형태를 한번에 확인할 수 있습니다.

```bash
$ ros2 service list -t | grep motion

/dsr01/motion/alter_motion [dsr_msgs2/srv/AlterMotion]
/dsr01/motion/change_operation_speed [dsr_msgs2/srv/ChangeOperationSpeed]
/dsr01/motion/check_motion [dsr_msgs2/srv/CheckMotion]
...
/dsr01/motion/move_joint [dsr_msgs2/srv/MoveJoint]
/dsr01/motion/move_jointx [dsr_msgs2/srv/MoveJointx]
/dsr01/motion/move_line [dsr_msgs2/srv/MoveLine]
...
/dsr01/motion/set_singularity_handling [dsr_msgs2/srv/SetSingularityHandling]
/dsr01/motion/trans [dsr_msgs2/srv/Trans]
```
</details>







<br/><br/>

## [arguments]
이러한 과정을 통해 [service_name]와 [service_type]를 알 수 있었습니다.<br/>
다음 과정으로, 커맨드에서 서비스를 호출하기 전, 입력 인수[arguments]의 구조를 알아야 합니다.<br/>

이는 Doosan Robotics ROS2 패키지의 소스 코드 또는 Github를 참고하여 확인할 수 있습니다.

> doosan-robot2/<br/>
> > dsr_msgs2/<br/>
> > >srv/<br/>
> > > > motion<br/>
> > > > > MoveHome.srv<br/>
> > > > > MoveJoint.srv<br/>
> > > > > MoveJointx.srv<br/>
> <br/>
<center>파일 위치 : doosan-robot2/dsr_msgs2/srv</center><br/>

```bash
#____________________________________________________________________________________________
# move_joint  
# The robot moves to the target joint position (pos) from the current joint position.
#____________________________________________________________________________________________

float64[6] pos               # target joint angle list [degree] 
float64    vel               # set velocity: [deg/sec]
float64    acc               # set acceleration: [deg/sec2]
float64    time #= 0.0       # Time [sec] 
float64    radius #=0.0      # Radius under blending mode [mm] 
int8       mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       blend_type #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       sync_type #=0      # SYNC = 0, ASYNC = 1
---
bool success
```
<center>입력인수의 구조 (MoveJoint.srv)</center><br/>







<br/><br/>

## 명령어
이를 통해 입력 인수[arguments]의 구조까지 확인하였고, 이를 바탕으로 서비스 요청 명령어를 작성합니다. <br/>
본 예제에서는 조인트 각도 [0, 0, 90, 0, 90, 0]로 이동하도록 서비스를 요청하겠습니다. <br/>
(입력인수는 YAML구문을 이용하여 작성합니다.) <br/>

```bash
[service_name] : /dsr01/motion/move_joint
[service_type] : dsr_msgs2/srv/MoveJoint
[arguments] : "{pos: [0, 0, 90, 0, 90, 0], vel: 60, acc: 60}"

$ ros2 service call [service_name] [service_type] [arguments]
$ ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0, 0, 90, 0, 90, 0], vel: 60, acc: 60}"
```

> <br/>
> <center>영상</center>
> <br/>

<center>service call 예시 영상</center><br/>

<br/>

<details>
<summary>서비스 세부 정보 확인 방법</summary>

<br/>

해당 서비스의 요청을 받고, 응답을 반환하는 노드가 무엇인지 확인할 수 있습니다.<br/>
아래 명령어를 통해 서비스”/dsr01/motion/move_joint”를 취급하는 노드가 있는지 확인합니다.<br/>

``` bash
$ for node in $(ros2 node list)
$ do
$ echo "Checking node: $node"
$ ros2 node info $node | grep '/dsr01/motion/move_joint' && echo " -> found in $node"
$ done

Checking node: /dsr01/connection_node
Checking node: /dsr01/controller_manager
Checking node: /dsr01/dsr_controller2
/dsr01/motion/move_joint: dsr_msgs2/srv/MoveJoint
/dsr01/motion/move_jointx: dsr_msgs2/srv/MoveJointx
-> found in /dsr01/dsr_controller2
Checking node: /dsr01/dsr_hw_interface2
Checking node: /dsr01/dsr_hw_interface_update
Checking node: /dsr01/robot_state_publisher
Checking node: /dsr01/rviz2
```

<br/>

노드“/dsr01/dsr_controller2”에서 서비스“/dsr01/motion/move_joint”를 취급하는 것을 확인할 수 있습니다.<br/>
이후 다음 명령어를 통해 노드의 정보를 확인하면, 해당 노드는 서비스”/dsr01/motion/move_joint”의 서비스 서버임을 확인할 수 있습니다.<br/>

``` bash
$ ros2 node info /dsr01/dsr_controller2

/dsr01/dsr_controller2
Subscribers:
/dsr01/alter_motion_stream: dsr_msgs2/msg/AlterMotionStream
...
/parameter_events: rcl_interfaces/msg/ParameterEvent
Publishers:
/dsr01/dsr_controller2/transition_event: lifecycle_msgs/msg/TransitionEvent
...
/rosout: rcl_interfaces/msg/Log
Service Servers:
/dsr01/aux_control/get_control_mode: dsr_msgs2/srv/GetControlMode
...
/dsr01/motion/move_joint: dsr_msgs2/srv/MoveJoint # 해당 서비스의 서비스 서버임을 확인할 수 있습니다.
...
/dsr01/tool/set_tool_shape: dsr_msgs2/srv/SetToolShape
Service Clients:

Action Servers:

Action Clients:

```
</details>







<br/><br/>

# 2) 토픽 발행

서비스를 요청하여 로봇에 명령을 전달한 것과 유사한 방식으로 토픽 또한 발행할 수 있습니다.<br/>
본 예제에서는 로봇에 조그이동 명령을 전달하는 토픽을 발행하겠습니다.<br/>
서비스 요청과 차이점으로는 발행 주기[rate]가 있습니다.<br/>

[rate] 위치에 --once를 입력하면 토픽을 한 번만 발행하고, --rate 1을 입력하면 1초에 한 번씩 발행합니다.<br/>
(숫자는 Hz를 의미, 1초에 몇 회 발행할지를 결정합니다.)<br/>
(발행 주기, 토픽 이름, 토픽 형태, 입력 인수)<br/>

```bash
$ ros2 topic pub [rate] [topic_name] [topic_type] [arguments]
```







<br/><br/>

## [topic_name]

```bash
1 $ ros2 topic list
2
3 /clicked_point
4 /dsr01/alter_motion_stream
5 /dsr01/dsr_controller2/joint_trajectory
6 ...
7 /dsr01/jog_multi
8 ...
9 /tf
10 /tf_static
11
```








<br/><br/>

## [topic_type]

```bash
$ ros2 topic type /dsr01/jog_multi

dsr_msgs2/msg/JogMultiAxis
```

<details>
<summary>토픽을 찾는 다른 방법</summary>

다른 방법으로는 아래 명령어를 통해 토픽의 이름과 형태를 한번에 확인할 수 있습니다.

``` bash
$ ros2 topic list -t

/clicked_point [geometry_msgs/msg/PointStamped]
/dsr01/alter_motion_stream [dsr_msgs2/msg/AlterMotionStream]
/dsr01/dsr_controller2/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
...
/dsr01/jog_multi [dsr_msgs2/msg/JogMultiAxis]
...
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

```
</details>








<br/><br/>

## [arguments]

> doosan-robot2/<br/>
> > dsr_msgs2/<br/>
> > >msg/<br/>
> > > > motion<br/>
> > > > > AlterMotionStream.msg<br/>
> > > > > JogMultiAxis.msg<br/>
> > > > > . . .<br/>
> <br/>
<center>파일 위치 : doosan-robot2/dsr_msgs2/msg</center><br/>

```bash
#____________________________________________________________________________________________
# multi jog
# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] 
#____________________________________________________________________________________________

float64[6]  jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
int8        move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
float64     speed             # jog speed [%]

```
<center>입력인수의 구조 (JogMultiAxis.msg)</center><br/>








<br/><br/>

## 명령어
이를 통해 입력 인수[arguments]의 구조까지 확인하였고, 이를 바탕으로 토픽 발행 명령어를 작성합니다. <br/>
(입력인수는 YAML구문을 이용하여작성합니다.)<br/>
본 예제에서는 Task 좌표계의 x축 방향으로 조그이동하는 토픽을 발행하겠습니다.<br/>

``` bash
[topic_name] : /dsr01/jog_multi
[topic_type] : dsr_msgs2/msg/JogMultiAxis
[arguments] : "{jog_axis: [1,0,0,0,0,0], move_reference: 0, speed: 50}"

$ ros2 topic pub [rate] [topic_name] [topic_type] [arguments]
$ ros2 topic pub --once /dsr01/jog_multi dsr_msgs2/msg/JogMultiAxis "{jog_axis: [[1,0,0,0,0,0]], move_reference: 0, speed: 50}"
```

> <br/>
> <center>영상</center>
> <br/>

<br/>

<center>topic pub 예시 영상</center><br/>


<br/>




<details>
<summary>토픽 세부 정보 확인 방법 1</summary>

<br/>

해당 토픽을 구독하는 노드가 무엇인지 확인할 수 있습니다.
아래 명령어를 통해 토픽”/dsr01/jog_multi”의 정보를 확인하여 1) 토픽의 형태와 2,3) 해당 토픽을 발행하고 구독하는 노드의 숫자를 알 수 있습니다.

```bash
$ ros2 topic info /dsr01/jog_multi

Type: dsr_msgs2/msg/JogMultiAxis
Publisher count: 0
Subscription count: 1
```

<br/>

아래 명령어를 통해 토픽”/dsr01/jog_multi”를 취급하는 노드가 있는지 확인합니다.

```bash
$ for node in $(ros2 node list)
$ do
$   echo "Checking node: $node"
$   ros2 node info $node | grep '/dsr01/jog_multi' && echo " -> found in $node"
$ done

Checking node: /dsr01/connection_node
Checking node: /dsr01/controller_manager
Checking node: /dsr01/dsr_controller2
    /dsr01/jog_multi: dsr_msgs2/msg/JogMultiAxis
    -> found in /dsr01/dsr_controller2
Checking node: /dsr01/dsr_hw_interface2
Checking node: /dsr01/dsr_hw_interface_update
Checking node: /dsr01/robot_state_publisher
Checking node: /dsr01/rviz2
```

<br/>

노드“/dsr01/dsr_controller2” 에서 토픽”/dsr01/jog_multi”를 취급하는 것을 확인할 수 있습니다.<br/>
이후 다음 명령어를 통해 노드의 정보를 확인하면, 해당 노드는 토픽”/dsr01/jog_multi”을 구독하는 것을 확인할 수 있습니다.<br/>

```bash
$ ros2 node info /dsr01/dsr_controller2

/dsr01/dsr_controller2
Subscribers:
/dsr01/alter_motion_stream: dsr_msgs2/msg/AlterMotionStream
/dsr01/jog_multi: dsr_msgs2/msg/JogMultiAxis # 해당 토픽을 구독하는 것을 확인할 수 있습니다.
...
/parameter_events: rcl_interfaces/msg/ParameterEvent
Publishers:
/dsr01/dsr_controller2/transition_event: lifecycle_msgs/msg/TransitionEvent
...
/rosout: rcl_interfaces/msg/Log
Service Servers:
/dsr01/aux_control/get_control_mode: dsr_msgs2/srv/GetControlMode
...
/dsr01/tool/set_tool_shape: dsr_msgs2/srv/SetToolShape
Service Clients:

Action Servers:

Action Clients:

```
</details>










<br/>

<details>
<summary>토픽 세부 정보 확인 방법 2</summary>

<br/>

ROS의 종합 GUI 툴인 RQt의 rqt_graph를 이용하여 노드간 연결상태를 확인할 수 있는데,
이를 통해 노드”/dsr01/dsr_controller2”가 토픽”/dsr01/jog_multi”를 구독하는 모습을 확인할 수 있습니다

```bash
$ rqt_graph
```

<br/><br/>

> <br/>
> <center>rqt_graph 설정 사진</center>
> <br/>
<center>rqt_graph 설정 방법</center>
<center>(그래프가 나타나지 않으면, 화살표 모양의 새로고침 버튼을 클릭합니다.)</center>

<br/><br/>

> <br/>
> <center>rqt_graph 그래프 사진</center>
> <br/>
<center>토픽"/dsr01/jog_multi"를 구독하고 있는 노드"/dsr01/dsr_controller2"</center>
<center>사각형 : topic / 원 : node</center>
</details>










<br/><br/>

# 3) 토픽 구독

현재 실행되고 있는 노드에서 발행하는 토픽을 CLI 명령어를 통해 구독하여 정보를 확인할 수 있습니다.
본 예제에서는 로봇 TCP의 현재 위치값을 나타내는 토픽을 구독하겠습니다.
토픽을 구독하는 명령어는 다음과 같습니다. (토픽 이름)

```bash
$ ros2 topic echo [topic_name]
```




<br/><br/>

## [topic_name]

```bash
$ ros2 topic list

/clicked_point
/dsr01/alter_motion_stream
/dsr01/dsr_controller2/joint_trajectory
…
/dsr01/msg/current_posx
…
/tf
/tf_static
```





<br/><br/>

## 명령어

```bash
[topic_name] : /dsr01/msg/current_posx

$ ros2 topic echo [topic_name]
$ ros2 topic echo /dsr01/msg/current_posx
```

> <br/>
> <center>영상</center>
> <br/>
