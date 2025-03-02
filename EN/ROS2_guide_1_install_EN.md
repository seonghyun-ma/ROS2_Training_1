# <center>1. Installation</center>

<br/>
<!------------------------------------------------------------------->

# 1) Linux Installation

<!------------------------------------------------------------------->
<details>
<summary>1. Download Ubuntu (Ubuntu 22.04.5 LTS (Jammy Jellyfish))</summary>

1. Download the ISO file via the link.(ubuntu-22.04.5-desktop-amd64.iso)  
https://releases.ubuntu.com/jammy/
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>2. Create Bootable USB</summary>

1) Prepare a bootable USB (4GB or larger)<br/>
2) Download Rufus: https://rufus.ie/en/<br/>
3) Connect the USB to your computer and run Rufus<br/>
4) Select the previously downloaded "ubuntu-22.04.5-desktop-amd64.iso" file, then click the 'Start' button<br/>
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>3. Partitioning</summary>

1. Windows Control Panel > Navigate to the Disk Management page to partition your drive<br/>
2. Right-click the partition and select "Shrink Volume"<br/>
3. Allocate enough disk space for the Linux installation<br/>

</details>


<br/>
<!------------------------------------------------------------------->
<details>
<summary>4. Change Boot Order (Using BIOS)</summary>

1. Shut down the Windows OS
2. Connect the bootable USB
3. Start the computer
4. Enter BIOS by pressing F2 (may vary depending on motherboard)
5. Navigate to the Security menu
6. Click Secure Boot Configuration
7. Change Secure Boot Option to Disabled (revert after Linux installation if not using Linux)
8. Go to the Boot menu
9. Set the Linux OS installation USB as the first priority and save (revert after installation)
10. Restart the computer
</details>


<br/>
<!------------------------------------------------------------------->
<details>
<summary>5. Start Linux</summary>

1. Welcome : English, install Ubuntu
2. Keyboard layout : English(US), English(US)
3. Updates and other software : Normal installation
4. Installation type : Something else - choose the free space and click "+"
5. Create partition : Size*, Primary, Beginning of this space, swap area  
> Size : 102,400MB (100GB) (adjust as required)  
> Type for the new partition : Primary  
> Location for the new partition : Beginning of this space  
> Use as : Ext4 journaling file system  
Mount point : /
1. Where are you? : Seoul
2. Who are you? : Set name and password (set in English, for this guide everything is set to 'asd')
3. Welcome to Ubuntu :  Wait for installation to complete
4. Installation Complete : Click Restart Now
5.  Online Accounts : Skip
6.  Livepatch : Next
7.  Help improve Ubuntu : No, don't send system info - Next
</details>


<br/><br/>
<!------------------------------------------------------------------->

# 2) Initial Setup

<!------------------------------------------------------------------->
<details>
<summary>1. Set up sudo (Administrator Privileges)</summary>

<br/>

Open the administrator settings page (using terminal):
``` bash
$ sudo visudo
```
Add the following lines to the opened file:
``` bash
# Replace 'asd' with your username

# Find the line that says 'root ALL=(ALL:ALL) ALL' and add the following line right below it
asd ALL=(ALL:ALL) ALL # Grants the user permission to use sudo commands

# Add the following line to the very bottom of the bashrc file
asd ALL=NOPASSWD: ALL # Command to bypass password input for sudo commands

# Press Ctrl + X (to save and exit)
```
</details>



<br/>
<!------------------------------------------------------------------->
<details>
<summary>2. Installation terminator</summary>

``` bash
$ sudo apt install terminator -y
```
</details>

<br/>
<!------------------------------------------------------------------->
<details>
<summary>3. Installation VS code</summary>

#### Download: Download the vscode.deb file from the provided link
https://code.visualstudio.com/docs/?dv=linux64_deb

#### Installation (Using Terminal):
(Replace "code_1.94.2-1728494015_amd64" with the actual name of the downloaded file)
``` bash
$ cd Downloads && sudo apt install ./code_1.94.2-1728494015_amd64.deb
```
</details>


<br/>
<!------------------------------------------------------------------->
<details>
<summary>4. Setting up bashrc</summary>

The bashrc file automatically runs when the terminal is launched. Through this process, we will specify commands to assist in using ROS2.  
#### Open the bashrc file:
```bash
$ code ~/.bashrc
```
#### Modify the bashrc file (Add content)
Add the following lines to the bottom of the .bashrc file and save:  

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

# After saving, restart the terminal for the changes to take effect (or run "source ~/.bashrc"):  
# The rw command specified above will only work correctly after ROS2 installation and package build.  
```
</details>











<br/><br/>
<!------------------------------------------------------------------->

# 3) Program Installation
Enter the following commands into the terminal to install ROS2, the Doosan package, and other programs.<br/> 
This process may take a long time.<br/>

<!------------------------------------------------------------------->
<details>
<summary>For virtual environments,</summary>
<br/> 

For virtual environments,
```bash
$ git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git
```
do not use this command.

<br/>

```bash
$ mkdir -p ~/ros2_ws/src
$ cp ~/Downloads/doosan-robot2-humble-devel.zip ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ unzip doosan-robot2-humble-devel.zip
$ mv doosan-robot2-humble-devel doosan-robot2
$ rm ~/ros2_ws/src/doosan-robot2-humble-devel.zip
```
Instead, use this command to download the Doosan ROS2 Package.

</details>





<br/>
<!------------------------------------------------------------------->
<details>
<summary>Installation Commands (Legacy version + training examples, this guide **INSTALLS IT** this way.)</summary>
<br/> 

> Note<br/> 
> 
> "$ROS_DISTRO" and "$USER" require the ROS2 version and your username.<br/>
> e.g. rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
>
> To use ROS2 with Version 3.x Controller, specify the build option:<br/>
> $ colcon build --cmake-args -DDRCF_VER=3<br/>
> <br/>




<br/>

Execute the following commands at once: 

```bash

##################### Setting Variables #####################
ROS_DISTRO=humble # ros2 distribution
USER=asd # user name
ws_name=ros2_ws # workspace name

##################### ROS2 Installation #####################
### Set UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

### Install ROS2 repository and dependencies
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools

##################### Docker Installation #####################
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world

##################### Doosan ROS2 Package Installation#####################
### Prerequisite installation elements before package installation
sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget
sudo apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group
sudo apt install ros-humble-moveit* -y

### install gazebo sim
$ echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz-image ros-humble-tf-transformations

### We recommand the /home/<user_home>/ros2_ws/src
mkdir -p ~/$ws_name/src
cd ~/$ws_name/src
git clone -b humble-devel https://github.com/seonghyun-ma/doosan-robot2.git
git clone -b humble https://github.com/ros-controls/gz_ros2_control
sudo rosdep init
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

### Install Doosan Robot Emulator
cd ~/$ws_name/src/doosan-robot2
sudo usermod -aG docker $USER
sudo ./install_emulator.sh

### Build settings
cd ~/$ws_name
source /opt/ros/humble/setup.bash
colcon build # v3 --> colcon build -DDRCF_VER=3
. install/setup.bash
```
</details>





<br/>
<!------------------------------------------------------------------->
<details>
<summary>Installation Commands (Latest version, this guide does not use the latest version.)</summary>
<br/> 

> Note<br/> 
> 
> "$ROS_DISTRO" and "$USER" require the ROS2 version and your username.<br/>
> e.g. rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
>
> To use ROS2 with Version 3.x Controller, specify the build option:<br/>
> $ colcon build --cmake-args -DDRCF_VER=3<br/>
> <br/>




<br/>

Execute the following commands at once: 

```bash

##################### Setting Variables #####################
ROS_DISTRO=humble # ros2 distribution
USER=asd # user name
ws_name=ros2_ws # workspace name

##################### ROS2 Installation #####################
### Set UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

### Install ROS2 repository and dependencies
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools

##################### Docker Installation #####################
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world

##################### Doosan ROS2 Package Installation#####################
### Prerequisite installation elements before package installation
sudo apt-get update
sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget
sudo apt-get install -y ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group
sudo apt install ros-humble-moveit* -y

### install gazebo sim
$ echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs ros-humble-ros-gz-sim ros-humble-ros-gz-image ros-humble-tf-transformations

### We recommand the /home/<user_home>/ros2_ws/src
mkdir -p ~/$ws_name/src
cd ~/$ws_name/src
git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git
git clone -b humble https://github.com/ros-controls/gz_ros2_control
sudo rosdep init
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

### Install Doosan Robot Emulator
cd ~/$ws_name/src/doosan-robot2
sudo usermod -aG docker $USER
sudo ./install_emulator.sh

### Build settings
cd ~/$ws_name
source /opt/ros/humble/setup.bash
colcon build # v3 --> colcon build -DDRCF_VER=3
. install/setup.bash
```
</details>



<br/>
<!------------------------------------------------------------------->
<details>
<summary>Post-Installation Tasks</summary>

<br/>

After the installation is complete, restart the system through the terminal (Command : reboot)
</details>











<br/><br/>
<!------------------------------------------------------------------->

# 4) 명령어 및 단축키

<details>
<summary>Linux Commands (Workspace Related)</summary>

| Command | Content | Remarks |
|-|-|-|
|cd         |Changing Workspace Locations               |$ cd [file_path]|
|cd /       |Move to the root directory                 ||
|cd ~       |Move to the home directory                 ||
|cd .       |Move to the current directory (refresh)    ||
|cd ..      |Move to the parent directory               |/app/bin/logs → /app/bin|
|cd -       |Move to the previous directory (go back)
|pwd        |Check the current directory
|mkdir      |Create a directory
|rmdir      |Delete a directory (only works if the directory is empty)
|rm         |Delete a file or directory                 |-r : Delete subdirectories
|           |                                           |-f : Force delete
|           |                                           |-i : Confirms before deletion
|ls         |Check the list of files or directories in the current directory    |$ ls -al
|           |                                           |-a : View all
|           |                                   |-l : Detailed information (owner, size, modification time, etc.)
|           |                                           |-S : Sort by size
|           |                                           |-h : Change units (KB, GB, etc.)
</details>








<br/>
<!------------------------------------------------------------------->
<details>
<summary>Linux Commands (Check Files)</summary>

| Command | Content | Remarks |
|-|-|-|
|cat    | Check short text file contents  | Can concatenate and display multiple files ($ cat [file1] [file2])
|more   | Check long text file contents   | Automatically exits after reading the file (more)
|less   | Check long text file contents   | Unlike "more", allows up and down navigation, exit with 'q' (less)
|head   | Display the first 10 lines | Can specify the number of lines with the -n option (head -n 5 [file_name])
|tail   | Display the last 10 lines  | Can specify the number of lines with the -n option (tail -n 5 [file_name])
</details>








<br/>
<!------------------------------------------------------------------->
<details>
<summary>Linux Commands (File Editors)</summary>

| Command | Content | Remarks |
|-|-|-|
|vi      | CLI text editor|
|vim     | Enhanced version of vi|
|nano    | CLI text editor with Ctrl commands|
|gedit   | GUI text editor (similar to Notepad)|
|code    | Using VS Code editor | Available after installing Visual Studio Code
</details>







<br/>
<!------------------------------------------------------------------->
<details>
<summary>Linux Commands (etc.)</summary>

| Command | Content | Remarks |
|-|-|-|
|clear      | Clear terminal screen         |
|echo       | Print to the screen           | $ echo 'hello world'
|alias|Create user-defind command| $ alias aa="[long command]" → Use the long command by entering 'aa' in the CLI
|unalias    | Delete user-defind command    |
|grep       | Search for specific words     | Use in command output : $ [command] \| grep [content]
|||Use on files : $ grep [content] [file_name]
</details>



<br/>
<!------------------------------------------------------------------->
<details>
<summary>Terminal Command</summary>

| Terminal Command | Content | Remarks |
|-|-|-|
|Ctrl + Alt + T     | Run terminal | The bashrc file is executed together when the terminal starts
|Ctrl + '-'         | Decrease font size
|Ctrl + Shift + '+' | Increase font size
|Ctrl + Shift + C   | Copy
|Ctrl + Shift + V   | Paste
|Ctrl + Shift + W   | Close one terminal window | Terminator only
|Ctrl + Shift + Q   | Close all terminal windows | Terminator only
|Ctrl + Shift + E   | Split vertically | Terminator only
|Ctrl + Shift + O   | Split horizontally | Terminator only
</details>



<br/>
<!------------------------------------------------------------------->
<details>
<summary>Related to ROS2</summary>

### Nodes
| Command | Description |
|-|-|
|ros2 node list | List of currently running nodes
|ros2 node info [node name]	| Check information of the specified node

### Topics
| Command | Description |
|-|-|
|ros2 topic list                    | List of currently active topics
|ros2 topic type [topic name]       | Check type of the specified topic
|ros2 topic info [topic name]       | Check information of the specified topic
|ros2 topic echo [topic name]       | Subscribe to the specified topic (continuously receives messages)
|ros2 topic pub --[publish rate] [topic name] [type] [arguments]| Publish to the specified topic

### Services
| Command | Description |
|-|-|
|ros2 service list                                      | List of currently available services
|ros2 service type [service name]                       | Check type of the specified service
|ros2 service call [service name] [type] [arguments]    | Call the specified service

### Actions
| Command | Description |
|-|-|
|ros2 action list                                       | List of currently available actions
|ros2 action info [action name]                         | Check information of the specified action
|ros2 action send_goal [action name] [type] [arguments] | Execute the specified action

### Interface (Message Types)
| Command | Description |
|-|-|
|ros2 interface show [message type]| Check interface of the type (for data input)
</details>

