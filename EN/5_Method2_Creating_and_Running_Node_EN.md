# <center>5. Method 2 : Creating and Running Node</center>







<br/>

## Table of Contents
1. Service Client Node (test_service.py)
2. Topic Subscription Node (test_topic.py)
3. Node Based on Python Interface (test_python.py)







<br/>

In this example, we will create nodes that communicate through messages to control the robot.<br/>
The executable files created in this example will be included in the test_pkg package created in "Chapter 3: Creating Packages".<br/>

<br/>

## Initial Setup
Move to the workspace and configure the ROS2 environment.

```bash
$ cd ~/ros2_ws # Default working directory
$ source /opt/ros/humble/local_setup.bash # Configure ROS 2 environment
$ source install/setup.bash # Configure user workspace environment
```








<br/><br/>
<!---------------------------------------------------------------->

# 1) Service Client Node (test_service.py)
We will create a service client node and an executable to request the MoveJoint service.<br/>
The node created through the executable will communicate commands to the robot by requesting services from the control node.<br/>

<br/>

## 1-1) Create File
Create a file named test_service.py in the test_pkg directory.<br/>

> test_pkg<br/>
> > launch<br/>
> > resource<br/>
> > test<br/>
> > test_pkg<br/>
> > > __ init __.py<br/>
> > > test_code.py<br/>
> > > test_service.py<br/>
> <br/>
<center>test_pkg/test_pkg/test_service.py</center>









<br/><br/>

## 1-2) Write Code
<details>
<summary>test_service.py</summary>

```python
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
import time

class MoveJointClient(Node):
    def __init__(self):
        super().__init__('move_joint_client')
        self.client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /dsr01/motion/move_joint...')
        self.get_logger().info('Service /dsr01/motion/move_joint available.')

    def send_request(self, pos, vel, acc):
        request = MoveJoint.Request()
        request.pos = pos
        request.vel = vel
        request.acc = acc
        self.get_logger().info(f'Sending request: pos={pos}, vel={vel}, acc={acc}')
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
        return future

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response received: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveJointClient()

    # First request
    future = node.send_request([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=50.0, acc=50.0)
    while not future.done(): # Wait for service response
        rclpy.spin_once(node, timeout_sec=0.1)

    # Wait for 3 seconds
    for i in range(3):
        print(f'{i+1} s'); time.sleep(1)

    # Second request
    future = node.send_request([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=50.0, acc=50.0)
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    # Shutdown
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>









<br/><br/>

## 1-3) Add Executable
Add the executable to the 'console_scripts' in the setup.py file.<br/>
(The executable name is test_service_exe)<br/>

```bash
# Format: "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
    "test_service_exe = test_pkg.test_service:main", # added
],
```









<br/><br/>

## 1-4) Build Package
After saving the written test_service.py and setup.py files, rebuild the package to apply the changes.<br/>
The build directory should be /ros2_ws (command: cd ~/ros2_ws).<br/>

```bash
$ colcon build --packages-select test_pkg
```









<br/><br/>

## 1-5) Test

```bash
$ ros2 run test_pkg test_service_exe
```

Executing the above command will cause the robot to carry out the given commands as shown in the video.









<br/><br/><br/>

# 2) Topic Subscription Node (test_topic.py)
Creating a topic subscription node is similar to creating a service client node.<br/>
We will create a topic subscription node to subscribe to the current_posx topic.<br/>
The node created through the executable will subscribe to the topic published by the hardware interface node and print it to the terminal.<br/>









<br/><br/>

## 2-1) Create File

> test_pkg<br/>
> > launch<br/>
> > resource<br/>
> > test<br/>
> > test_pkg<br/>
> > > __ init __.py<br/>
> > > test_code.py<br/>
> > > test_service.py<br/>
> > > test_topic.py<br/>
> <br/>
<center>test_pkg/test_pkg/test_topic.py</center>









<br/><br/>

## 2-2) Write Code

<details>
<summary>test_topic.py</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CurrentPosxSubscriber(Node):
    def __init__(self):
        super().__init__('current_posx_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dsr01/msg/current_posx',
            self.listener_callback,
            10 # QoS Depth
        )
        self.subscription
        self.get_logger().info('Subscribed to /dsr01/msg/current_posx')

    def listener_callback(self, msg):
        rounded_data = [round(value, 2) for value in msg.data]
        self.get_logger().info(f'Received data: {rounded_data}')

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosxSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>









<br/><br/>

## 2-3) Add Executable

```python
# Format: "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
    "test_service_exe = test_pkg.test_service:main",
    "test_topic_exe = test_pkg.test_topic:main", # 추가
],
```









<br/><br/>

## 2-4) Build Package
```bash
$ colcon build --packages-select test_pkg
```









<br/><br/>

## 2-5) Test
```bash
$ ros2 run test_pkg test_topic_exe
```










<br/><br/><br/>

# 3) Node Based on Python Interface (test_python.py)
Instead of directly defining a service client in the executable, we will create a node using the Python Interface.<br/>
Creating a node using the Python Interface follows a similar process to the previously demonstrated methods.<br/>
In this example, we will import functions implemented in common2 package's DSR_ROBOT2.py and use them to create the executable.<br/>
(Location: common2/imp/DSR_ROBOT2.py)<br/>










<br/><br/>

## 3-1) Create File

> test_pkg<br/>
> > launch<br/>
> > resource<br/>
> > test<br/>
> > test_pkg<br/>
> > > __ init __.py<br/>
> > > test_code.py<br/>
> > > test_service.py<br/>
> > > test_topic.py<br/>
> > > test_python.py<br/>
> <br/>
<center>test_pkg/test_pkg/test_python.py</center>










<br/><br/>

## 3-2) Write Code

<details>
<summary>test_python.py</summary>

```python
import rclpy
import time

def main(args=None):
    ROBOT_ID = 'dsr01'
    ROBOT_MODEL = 'm0609'

    import DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    rclpy.init(args=args) # Initialize ROS2 client
    node = rclpy.create_node('test_move_node', namespace=ROBOT_ID) # Create node
    DR_init.__dsr__node = node # Set the node in Doosan robot settings module

    try:
        from DSR_ROBOT2 import movej, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 실행 부분
    movej(posj(0, 0, 90, 0, 90, 0), vel=50, acc=50)

    for i in range(3):
        print(f'{i+1} s'); time.sleep(1)

    movej(posj(0, 0, 0, 0, 0, 0), vel=50, acc=50)

    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>

<details>
<summary>DSR_ROBOT2.py (movej function)</summary>


```python
def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
    return ret
def amovej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None):
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
    return ret    
def _movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, _async=0):
    # _pos
    _pos = get_posj(pos)

... (truncated) ...

        future = _ros2_movej.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

... (truncated) ...

```

</details>










<br/><br/>

## 3-3) Add Executable
```python
# Format: "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
    "test_service_exe = test_pkg.test_service:main",
    "test_topic_exe = test_pkg.test_topic:main",
    "test_python_exe = test_pkg.test_python:main", # 추가
],
```










<br/><br/>

## 3-4) Build Package
```bash
$ colcon build --packages-select test_pkg
```










<br/><br/>

## 3-5) Test
```bash
$ ros2 run test_pkg test_python_exe
```
