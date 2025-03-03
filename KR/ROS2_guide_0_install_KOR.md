# <center>0. 설치</center>

<br/>
<!------------------------------------------------------------------->

# 1) 리눅스 설치

<!------------------------------------------------------------------->
<details>
<summary>1. 우분투 다운로드 (Ubuntu 22.04.5 LTS (Jammy Jellyfish))</summary>

1. 해당 링크를 통해 iso 파일 다운로드 (ubuntu-22.04.5-desktop-amd64.iso)  
https://releases.ubuntu.com/jammy/
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>2. 부팅용 USB 구성</summary>

1) 부팅용 USB 준비 (4GB 이상)  
2) Rufus 다운로드 : https://rufus.ie/en/
3) 컴퓨터와 USB 연결 후 Rufus 실행     
4) 앞서 설치한 ubuntu-22.04.5-desktop-amd64.iso 파일을 선택 후 '시작'버튼 클릭     
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>3. 파티션 분리</summary>

1. 하드 디스트 파티션 만들기 및 포맷
2. 파티션 우클릭 후 볼륨 축소
3. 리눅스를 설치할 만큼의 디스크 할당
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>4. 부팅 순서 변경 (BIOS 사용)</summary>

1. 윈도우 OS 종료
2. 부팅용 USB 연결
3. 컴퓨터 시작
4. F2로 BIOS 진입 (메인보드 종류에 따라 상이)
5. Security 메뉴 진입
6. Secure Boot Configuration 클릭
7. Secure Boot Option을 Disabled로 변경 (리눅스 안 쓸때는 원상복귀)
8. Boot 메뉴 진입
9. 리눅스 OS 설치용 USB를 우선순위 1로 변경 후 저장 (설치 완료 후 원상복귀)
10. 재시작
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>5. 리눅스 시작</summary>

1. Welcome : English, install Ubuntu
2. Keyboard layout : English(US), English(US)
3. Updates and other software : Normal installation
4. Installation type : Something else - free space 선택 후 "+" 클릭
5. Create partition : Size*, Primary, Beginning of this space, swap area  
> Size : 102,400MB (100GB) (임의 설정)  
> Type for the new partition : Primary  
> Location for the new partition : Beginning of this space  
> Use as : Ext4 journaling file system  
Mount point : /
6. Where are you? : Seoul
7. Who are you? : name, password 설정 (영어로 설정, 본 가이드에서는 전부 'asd'로 설정)
8. Welcome to Ubuntu : 설치 기다리기
9. Installation Complete : Restart Now (재시작)
10. Online Accounts : Skip
11. Livepatch : Next
12. Help improve Ubuntu : No, don't send system info – Next
</details>

<br/><br/>
<!------------------------------------------------------------------->

# 2) 초기 설정

<!------------------------------------------------------------------->
<details>
<summary>1. sudo 설정 (관리자 권한)</summary>

관리자 설정 페이지 열기
``` bash
$ sudo visudo
```
열린 파일에 아래 내용 추가
``` bash
# asd 대신 사용자 이름 지정

# root ALL=(ALL:ALL) ALL 를 찾은 뒤, 이 바로 아래 해당 문구 추가
asd ALL=(ALL:ALL) ALL # sudo 명령어를 사용할 수 있는 권한 부여

# bashrc 파일 제일 아래부분에 해당 문구 추가
asd ALL=NOPASSWD: ALL # password 입력 생략

# Ctrl + x (저장 후 종료)
```
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>2. terminator 설치</summary>

``` bash
$ sudo apt install terminator -y
```
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>3. VS code 설치</summary>

#### 다운로드 : 해당 링크에서 vscode.deb 다운로드  
https://code.visualstudio.com/docs/?dv=linux64_deb

#### 설치 (터미널 사용)  
(code_1.94.2-1728494015_amd64 대신 직접 다운받은 파일의 이름 지정)
``` bash
$ cd Downloads && sudo apt install ./code_1.94.2-1728494015_amd64.deb
```
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>4. 한글 입력 설정</summary>

1. Settings - Region&Laguage - Manage Installed Language - Install/Remove Languages... - Korean - Apply - (설치) - 한국어확인 - close - roboot
2. Terminal - ibus-setup - Input Method - Add - Korean - Hangul - Add - Close
3. Settings - Keyboard - '+' - Korean - Korea (Hangul) - Add
4. 우측상단 - Hangul mode (on)
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>5. bashrc 파일 설정</summary>

bashrc는 terminal 실행시 자동으로 실행되는 파일이며, 해당 과정을 통해서 ROS2 사용을 보조하기 위한 명령어를 지정.  
#### bashrc 파일 열기
```bash
$ code ~/.bashrc
```
#### bashrc 수정 (내용추가) : .bashrc 파일 최하단에 다음 내용 추가 후 저장
```bash
echo "bashrc is reloaded!"
echo "==================="
echo "if you want to use ROS2(ID=13), type the command \"rt\""
echo "if you want to reload the bashrc, type the command \"sb\""
echo "if you want to open the bashrc, type the command \"cb\""
alias cb="code ~/.bashrc"
alias sb="source ~/.bashrc"
alias cb="code ~/.bashrc"
alias ros_domain="export ROS_DOMAIN_ID=13; echo \"ROS_DOMAIN_ID=13\""
rw() {
cd ~/ros2_ws
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/common2/lib/common2/imp
source /opt/ros/humble/local_setup.bash
source install/setup.bash
ros_domain
echo "ROS2 humble is activated! (ID=13) (ros2_ws)"
}
rw

# 저장 후, 터미널을 재시작해야 적용됨 (또는 source ~/.bashrc)
# 위에서 지정한 rw 명령어는 ROS2설치 및 PKG 빌드 이후에만 정상적으로 작동함
```
</details>













<br/><br/>
<!------------------------------------------------------------------->

# 3) 프로그램 설치
아래 명령어를 터미널에 입력하여 ROS2, Doosan pakage, 기타 프로그램을 설치하며, 많은 시간이 소요된다.

<br/>
<!------------------------------------------------------------------->
<details>
<summary>가상환경의 경우 참고</summary>
<br/> 

가상머신의 경우,
```bash
$ git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git
```
이 명령어를 사용하지 않고,
```bash
$ mkdir -p ~/ros2_ws/src
$ cp ~/Downloads/doosan-robot2-humble-devel.zip ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ unzip doosan-robot2-humble-devel.zip
$ mv doosan-robot2-humble-devel doosan-robot2
$ rm ~/ros2_ws/src/doosan-robot2-humble-devel.zip
```
이 명령어를 입력하여 Doosan ROS2 Pakage를 다운로드한다.

</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>설치 명령어</summary>
<br/> 

아래 내용 한번에 실행
> 참고사항<br/>
> 
> Line 49, 53에는 ROS2 version과 사용자이름을 입력합니다.<br/>
> e.g. rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
>
> To use ROS2 with Version 3.x Controller, specify the build option:<br/>
> $ colcon build --cmake-args -DDRCF_VER=3<br/>



```bash
##################### ROS2 Installation #####################
### Set UTF-8 locale
$ sudo apt update && sudo apt install -y locales
$ sudo locale-gen en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

### Install ROS2 repository and dependencies
$ sudo apt install -y software-properties-common curl
$ sudo add-apt-repository universe -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update
$ sudo apt upgrade -y
$ sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools

##################### Docker Installation #####################
$ sudo apt-get update
$ sudo apt-get install -y ca-certificates curl
$ sudo install -m 0755 -d /etc/apt/keyrings
$ sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
$ sudo chmod a+r /etc/apt/keyrings/docker.asc
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
$ sudo docker run hello-world

##################### Doosan ROS2 Package Installation#####################
### Prerequisite installation elements before package installation
$ sudo apt-get update
$ sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget
$ sudo apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group
$ sudo apt install ros-humble-moveit* -y

### install gazebo sim
$ echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz-image ros-humble-tf-transformations

### We recommand the /home/<user_home>/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git
$ git clone -b humble https://github.com/ros-controls/gz_ros2_control
$ sudo rosdep init
$ rosdep update
$ rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

### Install Doosan Robot Emulator
$ cd ~/ros2_ws/src/doosan-robot2
$ sudo usermod -aG docker $USER
$ sudo ./install_emulator.sh

### Build settings
$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ colcon build # v3 --> colcon build -DDRCF_VER=3
$ . install/setup.bash
```
</details>



<br/>
<!------------------------------------------------------------------->
<details>
<summary>설치 후 작업</summary>

<br/>

설치가 완료되면 터미널을 통해 재시작 (명령어 reboot)
</details>












<br/><br/>
<!------------------------------------------------------------------->

# 4) 명령어 및 단축키

<details>
<summary>리눅스 명령어 (작업공간 관련)</summary>

|   명령어  |    내용 |   비고 |
|-|-|-|
|cd|작업 공간 이동|$ cd [file_path]
|cd /|root 디렉토리로 이동||
|cd ~|홈 디렉토리로 이동||
|cd .|현재 디렉토리로 이동 (새로고침)||
|cd ..|상위 디렉토리로 이동|/app/bin/logs → /app/bin|
|cd -|이전 디렉토리로 이동 (뒤로가기)
|pwd|현재 디렉토리 확인
|mkdir|디렉토리 생성
|rmdir|디렉토리 삭제 (내부 파일이 없어야 실행 가능)
|rm|파일 또는 디렉토리 삭제| -r : 하위 디렉토리 삭제
|||-f : 강제로 삭제
|||-i : 파일 지울지 물어봄
|||-v : 삭제 정보 보여줌
|ls| 현재 디렉토리의 파일 또는 디렉토리 목록 확인 |$ ls -al
|||-a : 전부 확인
|||-l : 상세정보 (소유자, 크기, 수정시간 등)
|||-S : 크기별 정렬
|||-h : 단위 표현 변경 (KB, GB 등)
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>리눅스 명령어 (파일 확인)</summary>

|   명령어  |    내용 |   비고 |
|-|-|-|
|cat| 짧은 텍스트 파일 내용 확인| 여러 파일을 연결하여 출력 가능 ($ cat [파일1] [파일2])
|more| 긴 텍스트 파일 내용 확인| 파일을 다 읽으면 more 자동 종료
|less| 긴 텍스트 파일 내용 확인| more과는 다르게 위아래 이동 가능, q키를 눌러 종료
|head| 처음 10줄 출력| -n 옵션으로 라인 수 지정 가능 (head -n 5 [파일이름])
|tail| 마지막 10줄을 출력| -n 옵션으로 라인 수 지정 가능 (tail -n 5 [파일이름])
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>리눅스 명령어 (파일 편집기)</summary>

|   명령어  |    내용 |   비고 |
|-|-|-|
vi| CLI 텍스트 에디터|
vim| vi 향상 버전|
nano| Ctrl 명령어 사용 가능한 CLI 텍스트 에디터|
gedit| GUI 텍스트 에디터 (메모장과 유사)|
code| VS code 편집기 사용| Visual Studio Code 설치 후 사용 가능
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>리눅스 명령어 (기타)</summary>

|   명령어  |    내용 |   비고 |
|-|-|-|
|clear| terminal 화면 지움|
|echo| 화면 출력| $ echo 'hello world'
|alias| 사용자 명령어 생성| $ alias aa="[긴 명령어]" → CLI에 aa만 입력하여 긴 명령어를 사용 가능
|unalias| 사용자 명령어 삭제|
|grep| 특정 단어 검색| 명령어 출력 결과에 사용 : $ [명령어] | grep [내용]
|||파일 대상 사용 : $ grep [내용] [파일이름]
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>터미널 단축키</summary>

|   터미널 단축키  |    내용 |   비고 |
|-|-|-|
|Ctrl + Alt + T| 터미널 실행 |터미널 실행 시 bashrc 파일도 함께 실행
|Ctrl + '-'| 글자 크기 축소
|Ctrl + Shift + '+'| 글자 크기 확대
|Ctrl + Shift + C| 복사
|Ctrl + Shift + V| 붙여넣기
|Ctrl + Shift + W| 터미널 창 1개 종료 |terminator 전용
|Ctrl + Shift + Q| 터미널 전체 종료 |terminator 전용
|Ctrl + Shift + E| 좌우 분할 |terminator 전용
|Ctrl + Shift + O| 상하 분할 |terminator 전용
</details>


<br/>
<!------------------------------------------------------------------->
<details>
<summary>ROS2 관련</summary>

### 노드
|  명령어  |    내용 |
|-|-|
|ros2 node list| 현재 실핸중인 노드 목록
|ros2 node info| [노드 이름] 지정 노드의 정보 확인

### 토픽
|  명령어  |    내용 |
|-|-|
|ros2 topic list| 현재 동작중인 토픽 목록
|ros2 topic type [토픽 이름]| 지정 토픽의 타입 확인
|ros2 topic info [토픽 이름]| 지정 토픽의 정보 확인
|ros2 topic echo [토픽 이름]| 지정 토픽 구독 (계속 받아옴)
|ros2 topic pub --[발행주기] [토픽 이름] [타입] [입력 인자]| 지정 토픽 발행 (한번)

### 서비스
|  명령어  |    내용 |
|-|-|
|ros2 service list| 현재 제공되는 서비스 목록
|ros2 service type [서비스 이름]| 지정 서비스의 타입 확인
|ros2 service call [서비스 이름] [타입] [입력 인자]| 지정 서비스 실행

### 액션
|  명령어  |    내용 |
|-|-|
|ros2 action list| 현재 제공되는 액션 목록
|ros2 action info [액션 이름]| 지정 액션의 정보 확인
|ros2 action send_goal [액션 이름] [타입] [입력 인자]| 지정 액션 실행

### 인터페이스 (메시지 타입)
|  명령어  |    내용 |
|-|-|
|ros2 interface show [메시지 타입]| 타입의 인터페이스 확인 (데이터 입력용)
</details>
