ROS2로 두산 협동로봇을 조작하는 방법에는 크게 2가지가 있습니다.  
(CLI 명령어 사용, executable을 실행하여 노드 실행)  
본 가이드는 리눅스 설치부터 가상 로봇 조작까지의 내용을 포함하고 있습니다.  

### 진행 순서
1. 설치<br/>
ROS2, Docker, Doosan Package를 설치하는 방법
2. 시뮬레이션 실행<br/>
Doosan Robotics Github에서 제공하는 dsr_bringup2 패키지를 통해 시뮬레이션 실행
3. CLI 명령어 사용 (방법 1)<br/>
터미널 명령어를 통해 직접 명령 전송
4. 패키지 생성<br/>
executable을 위한 패키지 생성
5. 노드 생성 및 실행 (방법 2)<br/>
executable을 실행하여 노드 실행 (방법 2)


### 저작권
본 가이드의 모든 내용과 도안에 대한 저작권 및 지적재산권은 두산로보틱스에 있습니다. 따라서 두산로보틱스의 서면 허가 없이 사용, 복사, 유포하는 어떠한 행위도 금지됩니다. 또한 특허권을 오용하거나 변용하는데 따르는 책임은 전적으로 사용자에게 있습니다.<br/>

본 매뉴얼은 신뢰할 수 있는 정보지만 오류 또는 오탈자로 인한 손실에 대해 어떠한 책임도 지지 않습니다. 제품 개선에 따라 매뉴얼에 포함된 정보는 예고 없이 변경될 수 있습니다. <br/>
© Doosan Robotics Inc., All rights reserved<br/>

<br/>
<br/>
<p align="center">
  <img src="../image/0_Doosan_CI.png">
</p>
<!--![Alt text](image/ci-img01-2022.png)-->
<!--<img src="image/ci-img01-2022.png" width="650" height="400" />-->
