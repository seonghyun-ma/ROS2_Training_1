# 1. 시뮬레이션 실행

## 1) Launch 파일
> https://github.com/doosan-robotics/doosan-robot2/tree/humble-devel/dsr_bringup2/launch <br/>
> dsr_bringup2_gazebo.launch.py <br/>
> dsr_bringup2_moveit.launch.py <br/>
> dsr_bringup2_rviz.launch.py <br/>
> dsr_bringup2_spawn_on_gazebo.launch.py <br/>

<br/>

## 2) 입력 인자 확인
해당 launch 파일 중, 가상 로봇을 시각화하기위한 dsr_bringup2_rviz.launch.py 를 확인해보면, <br/>
다음과 같이 입력 인자를 통해 시뮬레이션 환경을 구성할 수 있습니다. <br/>

```python
def generate_launch_description():
    ARGUMENTS =[ 
        DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',  default_value = '127.0.0.1', description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',  default_value = 'virtual',   description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('gui',   default_value = 'false',     description = 'Start RViz2'    ),
        DeclareLaunchArgument('gz',    default_value = 'false',     description = 'USE GAZEBO SIM'    ),
        DeclareLaunchArgument('rt_host',    default_value = '192.168.137.50',     description = 'ROBOT_RT_IP'    ),
    ]
```
<center>dsr_bringup2_rviz.launch.py의 입력 인자</center>

<br/>

## 3) 초기 설정
작업 공간 이동 및 ROS2 환경 설정을 수행합니다.
``` bash
$ cd ~/ros2_ws # 기본 작업 위치
$ source /opt/ros/humble/local_setup.bash # ROS 2 환경 설정
$ source install/setup.bash # 사용자 워크스페이스 환경
```

<br/>

## 4) Launch 파일 실행
Launch 파일을 실행하는 방법은 다음과 같습니다.
```bash
$ ros2 launch [package_name] [launch_file_name] [arguments]
```
다음 아래의 명령어를 터미널에 입력하여 rviz를 이용한 가상로봇환경을 활성화할 수 있습니다.
```bash
$ ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=m0609
```

## 5) 실행 결과
> <br/>
> <br/>
> <center>동영상 입력</center>
> <br/>
> <br/>
<center>dsr_bringup2_rviz.launch.py 실행 결과 (rviz 실행 화면)</center>


<!------------------------------------------------------------------->

<!--
<details>
<summary>1. 우분투 다운로드 (Ubuntu 22.04.5 LTS (Jammy Jellyfish))</summary>

1. 해당 링크를 통해 iso 파일 다운로드 (ubuntu-22.04.5-desktop-amd64.iso)  
https://releases.ubuntu.com/jammy/

```bash

```

</details>
-->

<!--  -->
