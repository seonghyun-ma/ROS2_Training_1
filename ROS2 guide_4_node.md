# <center>4. 노드 생성 및 실행 (방법 2)</center>







<br/>

## 목차
1. 서비스 클라이언트 노드 (test_service.py)
2. 토픽 구독 노드 (test_topic.py)
3. Python Interface 기반 노드 (test_python.py)







<br/>

본 예제에서는 메시지 통신을 하는 노드를 생성하여 로봇을 제어하겠습니다.<br/>
본 예제에서 생성할 실행파일은 “3장 패키지 생성” 에서 생성한 test_pkg패키지에 생성합니다.<br/>

<br/>

## 초기 설정
작업 공간 이동 및 ROS2 환경 설정을 수행합니다.

```bash
$ cd ~/ros2_ws # 기본 작업 위치
$ source /opt/ros/humble/local_setup.bash # ROS 2 환경 설정
$ source install/setup.bash # 사용자 워크스페이스 환경
```








<br/><br/>
<!---------------------------------------------------------------->

# 1) 서비스 클라이언트 노드 (test_service.py)
서비스 클라이언트 노드를 생성하여 MoveJoint 서비스를 요청하는 실행파일(executable)을 생성하겠습니다.<br/>
실행파일을 통해 생성된 노드는 컨트롤 노드에 서비스를 요청하는 과정을 통해 로봇에 명령을 전달합니다.<br/>

<br/>

## 1-1) 파일 생성
test_pkg 디렉토리에 test_service.py라는 이름으로 파일을 생성합니다.<br/>

> <br/>
> <br/>
> <br/>
> <br/>
<center>test_pkg/test_pkg/test_service.py</center>









<br/><br/>

## 1-2) 코드 작성
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

    # 첫 번째 요청
    future = node.send_request([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=50.0, acc=50.0)
    while not future.done(): # 서비스 응답 대기
        rclpy.spin_once(node, timeout_sec=0.1)

    # 3초 대기
    for i in range(3):
        print(f'{i+1} s'); time.sleep(1)

    # 두 번째 요청
    future = node.send_request([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=50.0, acc=50.0)
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    # 종료
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>









<br/><br/>

## 1-3) executable 추가
setup.py파일의 'console_scripts'에 해당 실행파일을 추가합니다.<br/>
(executable 이름 : test_service_exe)<br/>

``` bash
# 형식 : "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
    "test_service_exe = test_pkg.test_service:main", # 추가
],
```









<br/><br/>

## 1-4) 패키지 빌드
작성된 test_service.py, setup.py 파일을 저장 후, 패키지를 다시 빌드해야 변경사항이 적용됩니다.<br/>
이 때, 빌드 위치는 /ros2_ws 입니다. (명령어 : cd ~/ros2_ws)<br/>

```bash
$ colcon build --packages-select test_pkg
```









<br/><br/>

## 1-5) 테스트

```bash
$ ros2 run test_pkg test_service_exe
```

다음 명령어를 실행시키면 영상처럼 로봇이 주어진 명령을 수행합니다.

> <br/>
> <br/>
> <br/>
> <br/>









<br/><br/><br/>

# 2) 토픽 구독 노드 (test_topic.py)
토픽 구독 노드 생성 또한 서비스 클라이언트 노드 생성 방법과 유사합니다.<br/>
토픽 구독 노드를 생성하여 current_posx 토픽을 구독하는 실행파일을 생성하겠습니다.<br/>
실행파일을 통해 생성된 노드는 하드웨어 인터페이스 노드에서 발행하는 토픽을 구독하여 터미널에 출력합니다.<br/>









<br/><br/>

## 2-1) 파일 생성

> <br/>
> <br/>
> <br/>
> <br/>
<center>test_pkg/test_pkg/test_topic.py</center>









<br/><br/>

## 2-2) 코드 작성

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

## 2-3) executable 추가

```python
# 형식 : "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
    "test_service_exe = test_pkg.test_service:main",
    "test_topic_exe = test_pkg.test_topic:main", # 추가
],
```









<br/><br/>

## 2-4) 패키지 빌드
```bash
$ colcon build --packages-select test_pkg
```









<br/><br/>

## 2-5) 테스트
```bash
$ ros2 run test_pkg test_topic_exe
```

> <br/>
> <br/>
> <br/>
> <br/>










<br/><br/><br/>

# 3) Python Interface 기반 노드 (test_python.py)
실행파일에서 service client를 직접 정의하는 대신에 Python Interface를 이용하여 노드를 생성해보겠습니다.<br/>
Python Interface를 이용한 노드 생성 방법 또한 앞서 진행한 노드 생성 방법과 유사한 방법으로 진행됩니다.<br/>
본 예제에서는 common2 패키지의 DSR_ROBOT2.py 에서 구현된 함수를 import한 뒤, 이를 이용하여 실행 파일을 구성합니다.<br/>
(위치 : common2/imp/DSR_ROBOT2.py)<br/>










<br/><br/>

## 3-1) 파일 생성

> <br/>
> <br/>
> <br/>
> <br/>
<center>test_pkg/test_pkg/test_python.py</center>










<br/><br/>

## 3-2) 코드 작성

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

    rclpy.init(args=args) # ROS2 클라이언트 초기화
    node = rclpy.create_node('test_move_node', namespace=ROBOT_ID) # 노드생성
    DR_init.__dsr__node = node # 두산 로봇 설정 모듈에 노드 설정

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
<summary>DSR_ROBOT2.py (movej 함수 부분)</summary>

<!-- @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ -->

```python
def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None,) # @
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=0)
    return ret
def amovej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None) # @
    ret = _movej(pos, vel, acc, time, radius, mod, ra, v, a, t, r, _async=1)
    return ret
def _movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None) # @
    # _pos
    _pos = get_posj(pos)

... (중략) ...

        future = _ros2_movej.call_async(req)
        rclpy.spin_until_future_complete(g_node, future)

... (이하 생략) ...

```
<!-- @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ -->
</details>










<br/><br/>

## 3-3) executable 추가
```python
# 형식 : "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
    "test_service_exe = test_pkg.test_service:main",
    "test_topic_exe = test_pkg.test_topic:main",
    "test_python_exe = test_pkg.test_python:main", # 추가
],
```










<br/><br/>

## 3-4) 패키지 빌드
```bash
$ colcon build --packages-select test_pkg
```










<br/><br/>

## 3-5) 테스트
```bash
$ ros2 run test_pkg test_python_exe
```

> <br/>
> <br/>
> <br/>
> <br/>
