# <center>4. Create Packages</center>







<br/>

## Table of Contents
1. Creating a Package
2. Creating Executables
3. Creating Executables (Node Creation)







<br/>

In ROS2, executables are used to create and run nodes, and they need to be included in a package.<br/>
In this section, we will introduce how to create a package and an executable.<br/>




<br/><br/>
<!---------------------------------------------------------------->

# 1) Creating a Package

<br/>

## 1-1) Initial Setup
Use the following commands to configure the ROS2 environment and move to the directory where the package will be created.

```bash
$ cd ~/ros2_ws # Default working directory
$ source /opt/ros/humble/local_setup.bash # Configure ROS 2 environment
$ source install/setup.bash # Configure user workspace environment
$ cd ~/ros2_ws/src/doosan-robot2 # Directory to create the package
```








<br/><br/>

## 1-2) Creating a Package
The command to create a package is as follows.

``` bash
$ ros2 pkg create [package name] --build-type [build type]
```

<details>
<summary>[Build Type]</summary>

ament_cmake: C++ package<br/>
ament_python: Python package<br/>
</details>

<br/><br/>

Create the package with the following command.
(The package will be created in the directory: ~/ros2_ws/src/doosan-robot2/test_pkg)

``` bash
$ ros2 pkg create test_pkg --build-type ament_python
```

> ros2_ws<br/>
> > src<br/>
> > >doosan-robot2<br/>
> > > > coomon2<br/>
> > > > dsr_bringup2<br/>
> > > > . . .<br/>
> > > > test_pkg<br/>
> <br/>
<center>Package "test_pkg" created in the specified directory</center>







<br/><br/>

<details>
<summary>launch</summary>

+) Modifying Package Files
To allow the package to handle launch files, you need to add a launch directory.<br/>
This process involves creating a launch directory and modifying the setup.py file accordingly.<br/>
(The created launch directory will be located at ~/ros2_ws/src/doosan-robot2/test_pkg/launch)<br/>

```bash
$ mkdir test_pkg/launch
```

<br/>

Next, modify the test_pkg/setup.py file as follows.
(Location of setup file: ~/ros2_ws/src/doosan-robot2/test_pkg/setup.py)

<details>
<summary>launch / Code Modifications (lines 1, 2, 15)</summary>

```python
import os ## added
from glob import glob ## added
from setuptools import find_packages, setup

package_name = 'test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')) ## added
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asd',
    maintainer_email='asd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```
</details>

</details>









<br/><br/>

## 1-3) Build
Next, build the package with the following commands.
```bash
$ cd ~/ros2_ws # Default working directory
$ colcon build --packages-select test_pkg
```








<br/><br/>

## 1-4) Test
The package creation is complete.<br/>
After restarting the terminal or re-executing the bashrc file, verify that the package has been successfully built.<br/>

```bash
1 $ source ~/.bashrc # Command to re-source the bashrc
2 $ ros2 pkg list | grep test_pkg
3
4 test_pkg
```









<br/><br/><br/>

# 2) Creating Executables
Next, we will create an executable within the test_pkg package.<br/>
Within the package directory, there is a directory with the same name as the package. The executable will be created in this directory.<br/>
(Location of this directory: ~/ros2_ws/src/doosan-robot2/test_pkg/test_pkg)<br/>

> test_pkg<br/>
> > resource<br/>
> > test<br/>
> > test_pkg<br/>
> 
> > package.xml<br/>
> > setup.cfg<br/>
> > setup.py<br/>
> <br/>








<br/><br/>

## 2-1) Create File
Create an executable file in the test_pkg directory using a text editor like VS Code.
In this example, we will create a file named test_code.py.

> test_pkg<br/>
> > resource<br/>
> > test<br/>
> > test_pkg<br/>
> > > __ init __.py<br/>
> > > test_code.py<br/>
> <br/>
<center>test_pkg/test_pkg/test_code.py</center>








<br/><br/>

## 2-2) Write Code
In this example, we will create a simple executable that prints the current time.<br/>
Enter the following code into the test_code.py file.<br/>
(Location: ~/ros2_ws/src/doosan-robot2/test_pkg/test_pkg/test_code.py)<br/>


<details>
<summary>test_code.py</summary>

```python
from datetime import datetime

def get_time():
    now = datetime.now()
    formatted_time = now.strftime("%Y-%m-%d %H:%M:%S")
    return formatted_time

def main(args=None):
    print('#####################################')
    print("Time the node was run :", get_time())
    print('#####################################')

if __name__ == '__main__':
    main()
```
</details>

<br/><br/>

Once the code is written, add the executable to the 'console_scripts' in the setup.py file.<br/>
The executable will be named test_code_exe.<br/>

```python
# Format : "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
],
```









<br/><br/>

## 2-3) Build
With the executable code written and the setup.py file configured, build the test_pkg package using the following commands.<br/>
The working directory should be ~/ros2_ws (command: cd ~/ros2_ws).<br/>

```bash
$ colcon build --packages-select test_pkg
```










<br/><br/>

## 2-4) Test
Once the build is complete, run the test_code_exe of the test_pkg package.
If it runs successfully, the current date and time will be displayed.

```bash
$ ros2 run test_pkg test_code_exe

#####################################
Time the node was run : 0000-00-00 00:00:00
#####################################
```









<br/><br/>

# 3) Creating Executables (Node Creation)
So far, we have been running a simple executable, not creating a node.<br/>
In this section, we will introduce how to create a node.<br/>









<br/>

## 3-1) Modify Code (to create a node)
Modify the test_code.py file as follows to create a node.

<details>
<summary>test_code.py (Modified)</summary>

```python
import rclpy # add
from rclpy.node import Node # add
from datetime import datetime

def get_time():
    now = datetime.now()
    formatted_time = now.strftime("%Y-%m-%d %H:%M:%S")
    return formatted_time

class TimeLoggerNode(Node): # add class
    def __init__(self):
        super().__init__('test_timer_node') # Node name: test_timer_node
        self.get_logger().info('#####################################')
        self.get_logger().info(f"Time the node was run: {get_time()}")
        self.get_logger().info('#####################################')

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2
    node = TimeLoggerNode() # Create node
    try:
        rclpy.spin(node) # Execute node
    except KeyboardInterrupt: # Terminate node with "Ctrl + C"
        node.get_logger().info('Node is shutting down.')
    finally:
        node.destroy_node() # Clean up after node termination
        rclpy.shutdown() # Terminate ROS 2

if __name__ == '__main__':
    main()
```
</details>










<br/><br/>

## 3-2) Test: Creating and Running a Node
Open two terminals and run the modified test_code.py in the first terminal.<br/>
We can confirm that the node is created and the current time is recorded in the log as shown below.<br/>

```bash
# First Terminal
$ ros2 run test_pkg test_code_exe

[INFO] [0000000000.000000000] [test_timer_node]: #####################################
[INFO] [0000000000.000000000] [test_timer_node]: Time the node was run: 0000-00-00 00:00:00
[INFO] [0000000000.000000000] [test_timer_node]: #####################################
```

<br/>

you will notice that the node is running but not exiting.<br/>
At this point, if you enter the following command in the second terminal, you will see that the test_timer_node node has been created.<br/>


```bash
# Second Terminal
$ ros2 node list | grep test_timer_node

/test_timer_node
```










<br/><br/>

## 3-3) Test: Ending the Node and Verification
Now, click the first terminal where the node is running and press "Ctrl + C" to terminate the test_timer_node node.
Then, search for the node again, and you will find that it no longer exists.

```bash
# First Terminal: Terminate with "Ctrl + C"
^C[INFO] [0000000000.000000000] [test_timer_node]: Node is shutting down.

# Second Terminal: No output as the node no longer exists
$ ros2 node list | grep test_timer_node
```
