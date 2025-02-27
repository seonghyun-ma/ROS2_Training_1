# <center>3. 패키지 생성</center>







<br/>

## 목차
1. 패키지 생성
2. 실행파일(executable) 생성
3. 실행파일(executable) 생성 (노드 생성)







<br/>

ROS2에서는 노드를 생성하고 실행하기 위해 실행파일(executable)을 사용하는데, 이를 포함할 패키지를 구성해야합니다. <br/>
따라서 해당 페이지에서는 패키지와 실행파일을 생성하는 방법을 소개하겠습니다.<br/>







<br/><br/>
<!---------------------------------------------------------------->

# 1) 패키지 생성

<br/>

## 1-1) 초기 설정
다음 명령어를 통해 ROS2 환경을 설정하고, 패키지를 생성할 위치로 이동합니다

```bash
$ cd ~/ros2_ws # 기본 작업 위치
$ source /opt/ros/humble/local_setup.bash # ROS 2 환경 설정
$ source install/setup.bash # 사용자 워크스페이스 환경
$ cd ~/ros2_ws/src/doosan-robot2 # 패키지 생성 위치
```








<br/><br/>

## 1-2) 패키지 생성
패키지를 생성하는 명령어는 다음과 같습니다.

``` bash
$ ros2 pkg create [패키지 이름] --build-type [빌드 타입]
```

<details>
<summary>[빌드 타입]</summary>

ament_cmake : C++ 패키지<br/>
ament_python : 파이썬 패키지<br/>
</details>

<br/><br/>

다음 명령어를 통해 패키지를 생성합니다.
(생성된 패키지 위치 : ~/ros2_ws/src/doosan-robot2/test_pkg)

``` bash
$ ros2 pkg create test_pkg --build-type ament_python
```

> <br/>
> <br/>
> <br/>
> <br/>
<center>지정한 위치에 패키지 “test_pkg”가 생성된 모습</center>







<br/><br/>

<details>
<summary>launch</summary>

+) 패키지 파일 수정
패키지가 launch 파일을 다룰 수 있도록 하기 위해서는 launch 디렉토리를 따로 추가해야합니다.<br/>
해당 과정은 launch 디렉토리를 추가하고, 이에 맞게 setup.py를 수정하는 과정입니다.<br/>
(생성된 launch 디렉토리 위치 : ~/ros2_ws/src/doosan-robot2/test_pkg/launch)<br/>

```bash
$ mkdir test_pkg/launch
```

<br/>

다음으로는 test_pkg/setup.py 파일을 다음과 같이 수정합니다.
(setup 파일 위치 : ~/ros2_ws/src/doosan-robot2/test_pkg/setup.py)

<details>
<summary>launch / 수정 내용 (code) (line 1, 2, 15)</summary>

```python
import os ## 추가
from glob import glob ## 추가
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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')) ## 추가
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

## 1-3) 빌드
이후 다음 명령어를 통해 패키지를 빌드합니다.
```bash
$ cd ~/ros2_ws # 기본 작업 위치
$ colcon build --packages-select test_pkg
```








<br/><br/>

## 1-4) 테스트
패키지 생성이 완료되었습니다.<br/>
터미널을 재실행하거나 bashrc 파일을 재실행한 후, 패키지가 정상적으로 빌드되었는지 확인합니다.<br/>

```bash
1 $ source ~/.bashrc # bashrc 를 재실행하는 명령어
2 $ ros2 pkg list | grep test_pkg
3
4 test_pkg
```









<br/><br/><br/>

# 2) 실행파일(executable) 생성
다음으로는 실행파일을 test_pkg 패키지 안에 생성하겠습니다.<br/>
패키지 디렉토리안에 패키지와 동일한 이름의 디렉토리가 있는데, 실행파일은 해당 디렉토리에 생성합니다.<br/>
(해당 디렉토리 위치 : ~/ros2_ws/src/doosan-robot2/test_pkg/test_pkg)<br/>

> <br/>
> <br/>
> <br/>
> <br/>









<br/><br/>

## 2-1) 파일 생성
vscode 등을 이용하여 test_pkg 디렉토리에 실행파일을 생성합니다.
본 예제에서는 test_code.py 라는 이름의 파일을 생성하겠습니다.

> <br/>
> <br/>
> <br/>
> <br/>
<center>test_pkg/test_pkg/test_code.py</center>








<br/><br/>

## 2-2) 코드 작성
본 예제에서는 현재 시간을 출력하는 간단한 실행파일을 생성하겠습니다.<br/>
test_code.py 파일에 다음 코드를 입력해줍니다.<br/>
(위치 : ~/ros2_ws/src/doosan-robot2/test_pkg/test_pkg/test_code.py<br/>


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

코드 작성이 완료되었으면, setup.py파일의 'console_scripts'에 executable을 추가합니다.<br/>
executable의 이름은 test_code_exe로 지정하였습니다.<br/>

```python
# 형식 : "[executable_name] = [package_name].[file_name]:main"

'console_scripts': [
    "test_code_exe = test_pkg.test_code:main",
],
```









<br/><br/>

## 2-3) 빌드
executable코드 작성과 setup.py파일 설정이 완료되었으면, 다음 명령어로 test_pkg패키지를 빌드합니다.<br/>
이때, 작업위치는 “~/ros2_ws” 입니다. (명령어 cd ~/ros2_ws)<br/>

```bash
$ colcon build --packages-select test_pkg
```









<br/><br/>

## 2-4) 테스트
빌드가 완료되었으면, test_pkg의 test_code_exe를 실행합니다.
정상적으로 실행될 경우, 현재 날짜와 시간이 표시됩니다.

```bash
$ ros2 run test_pkg test_code_exe

#####################################
Time the node was run : 0000-00-00 00:00:00
#####################################
```









<br/><br/>

# 3) 실행파일(executable) 생성 (노드 생성)
지금까지는 노드를 생성하는 것이 아닌, 단순히 executable을 실행하는 예제였습니다.<br/>
때문에 이 페이지에서 노드를 생성하는 방법까지 소개하도록 하겠습니다.<br/>









<br/>

## 3-1) 코드 수정 (노드를 생성하는 형식으로 수정)
다음 코드와 같이 test_code.py를 수정합니다.

<details>
<summary>test_code.py (수정)</summary>

```python
import rclpy # 추가
from rclpy.node import Node # 추가
from datetime import datetime

def get_time():
    now = datetime.now()
    formatted_time = now.strftime("%Y-%m-%d %H:%M:%S")
    return formatted_time

class TimeLoggerNode(Node): # 함수 추가
    def __init__(self):
        super().__init__('test_timer_node') # 노드이름 : test_timer_node
        self.get_logger().info('#####################################')
        self.get_logger().info(f"Time the node was run: {get_time()}")
        self.get_logger().info('#####################################')

def main(args=None):
    rclpy.init(args=args) # ROS 2 초기화
    node = TimeLoggerNode() # 노드 생성
    try:
        rclpy.spin(node) # 노드 실행
    except KeyboardInterrupt: # "Ctrl + C"를 눌러서 노드 종료
        node.get_logger().info('Node is shutting down.')
    finally:
        node.destroy_node() # 노드 종료 후 정리
        rclpy.shutdown() # ROS 2 종료

if __name__ == '__main__':
    main()
```
</details>










<br/><br/>

## 3-2) 테스트 : 노드 생성 및 실행
2개의 터미널을 연 다음에, 첫 번째 터미널로 수정된 test_code.py를 실행합니다.<br/>
그러면 다음과 같이 로그에 현재 시간이 기록되고, 노드가 생성되는 것을 확인할 수 있습니다.<br/>

```bash
# 첫 번째 터미널
$ ros2 run test_pkg test_code_exe

[INFO] [0000000000.000000000] [test_timer_node]: #####################################
[INFO] [0000000000.000000000] [test_timer_node]: Time the node was run: 0000-00-00 00:00:00
[INFO] [0000000000.000000000] [test_timer_node]: #####################################
```

<br/>

노드가 실행되었지만 종료되지 않는 것을 확인할 수 있습니다.<br/>
이 때 두 번째 터미널에 다음 명령어를 입력하면 test_timer_node 노드가 생성되어있는 것을 확인할 수 있습니다.<br/>


```bash
# 두 번째 터미널
$ ros2 node list | grep test_timer_node

/test_timer_node
```










<br/><br/>

## 3-3) 테스트 : 노드 종료 후 확인
이제 노드가 실행중인 첫 번째 터미널을 클릭한 뒤 "Ctrl + C"를 눌러서 test_timer_node 노드를 종료합니다.
그리고 다시 노드를 검색하면 노드가 없어진 것을 확인할 수 있습니다.

``` bash
# 첫 번째 터미널 : "Ctrl + C"를 눌러서 종료
^C[INFO] [0000000000.000000000] [test_timer_node]: Node is shutting down.

# 두 번째 터미널 : 노드가 없으므로 아무런 값도 나오지 않음
$ ros2 node list | grep test_timer_node
```
