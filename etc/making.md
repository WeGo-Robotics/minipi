# 축구 동작 패키지 모음

기초 작업 중...



---
### Imu를 사용한 영상 흔들림 보정
mini pi에서 imu는 200hz로 동작함.
영상을 40fps로 설정하면, 한 프레임 당 약 5개의 imu 값을 얻게 됨.
따라서, 영상 framerate를 40fps로 설정한 후, 한 프레임 당 모이는 5개의 imu 값의 쿼터니언(또는 오일러 각) 값을 평균냄.

=> 그럴 필요 없이 rosbag으로 기록해놓으면 됨

```bash
$ rosbag record -O run_2025-10-02_2.bag --split --size=4096 -b 2048 \
  /usb_cam/image_raw /camera/camera_info \
  /imu/original_data /imu/data \
  /yesense/imu_status /yesense/gyro_bias_estimate
```

---
### 조이스틱 관련 설정
현재 문제 상황

조이스틱으로 cmd_vel을 설정하기 때문에 코드로 전달하면 충돌 발생
따라서, remap을 통해 cmd_vel_test로 우회
그 경우, 조이스틱으로 속도조절을 할 수 없고, 제자리걷기/정지만 가능

=> 제자리걷기/정지를 코드로 전달할 때, 조이스틱에서도 보내기 때문에 subscribe 충돌 발생
코드에서, /usr2ctrl을 통해 제어하면 제자기걷기/정지 뿐만 아니라, cmd_vel도 remap 없이 깔끔하게 전달
그러나, 조이스틱보다 우선시되므로 코드를 실행하는 순간 조이스틱으로 제어 불가 => 비상 정지를 하려면 rostopic pub 또는 물리적으로 모터를 꺼야 함

필요한 것
**soccer를 위한 조이스틱 매핑 코드 또는 GUI**
GUI가 간편해 보임.

---
### twist mux를 이용한 다중 입력 cmd_vel (작성중)

twist_mux로 cmd_vel remap 없이 사용 가능해도 joy의 running_standby_switch 때문에 로봇의 완전한 정지는 어려움
아니면 usr2ctrl로 조종하고 코드 끌 때 rosnode kill (ball_tracking node)해도 됨.
그래도 중간에 estop은 들어가야 함....

일단 joy_stick(cmd_vel/joy)과 depth 카메라 기반 벽 회피(cmd_vel/wall)을 mux함. 기존 linear.x, angular.z를 0.0으로 보내도 우선순위에 걸리므로 완전히 타임아웃 시켜야 함 => if 문 사용하여 벽 회피 할 상황에서만 pub, 아니면 pass



---
### 다른 곳에서 받았을 때 설치해야 할 것 (작성 중)

yolo 로그 에러 => pi48에서 /.ros/py_logging_nolevel.ini 복붙
realsense_ws만들어서 거기에서 ros-realsense(ros 버전 맞게) 설치
librealsense
rknnlite2
twist-mux => sim2real_master/install/share/sim2real_master/joy_config.yaml에서 cmd_vel에 /joy 추가


용량 차면 아래 명령어로 로그 내역 삭제
```bash
# 비우기
$ sudo truncate -s 0 ~/var/log/uvcdynctrl-udev.log

# 규칙 파일 이동
$ sudo mv /lib/udev/rules.d/80-uvcdynctrl.rules /lib/udev/rules.d/80-uvcdynctrl.rules.DISABLED

# 적용
$ sudo udevadm control --reload-rules
$ sudo rm /var/log/uvcdynctrl-udev.log
```


---
#### realsense 관련 설정



이 오류는 `realsense-ros` 패키지를 빌드하는 과정에서 필요한 **외부 종속성 패키지**인 \*\*`ddynamic_reconfigure`\*\*가 시스템에 설치되어 있지 않기 때문에 발생합니다.

`ddynamic_reconfigure`는 ROS의 **동적 재설정(Dynamic Reconfigure)** 기능을 지원하는 데 필요한 라이브러리입니다. 소스 코드를 빌드하려면 이 패키지의 개발 파일이 반드시 필요합니다.

-----

## 🛠️ 해결책: 누락된 종속성 설치

ROS Noetic 환경에서 이 문제를 해결하는 가장 간단한 방법은 `apt` 패키지 관리자를 통해 해당 라이브러리를 설치하는 것입니다.

### 1\. 누락된 ROS 패키지 설치

터미널에서 다음 명령어를 실행하여 \*\*`ddynamic_reconfigure`\*\*를 설치하세요.

```bash
# ROS Noetic용 ddynamic_reconfigure 패키지 설치
sudo apt install ros-noetic-ddynamic-reconfigure
```

### 2\. ROS 종속성 재확인 (선택 사항이지만 권장)

이전에 `rosdep install`을 실행했지만, 시스템에 일부 종속성이 누락된 상태였을 수 있습니다. 위 패키지를 설치한 후, 워크스페이스의 모든 종속성을 다시 한 번 확인하고 설치를 시도하세요.

```bash
cd ~/catkin_ws  # 또는 사용 중인 워크스페이스 경로
rosdep install --from-paths src --ignore-src -r -y
```

### 3\. 다시 빌드 시도

종속성을 설치한 후에는 `catkin_make` 명령을 다시 실행하여 RealSense ROS 래퍼를 빌드합니다.

```bash
catkin_make
```

이 과정을 거치면 `ddynamic_reconfigure` 오류는 사라지고 빌드가 정상적으로 진행될 것입니다.


---
## 실행해야 하는 코드


1. 기본 joy_control.launch
```bash
$ cd ~/sim2real_master
$ source install/setup.bash
$ roslaunch sim2real_master joy_control.launch
```

2. twist_mux 실행
```bash
$ cd ~/soccer_ws
$ source devel/setup.bash
$ roslaunch wego_twist_mux twist_mux.launch
```

3. 광각 카메라 실행
```bash
$ rosrun usb_cam usb_cam_node _pixel_format:=yuyv
```
4. depth 카메라 실행
```bash
$ cd ~/realsense_ws
$ source devel/setup.bash
$ roslaunch realsense2_camera rs_camera.launch width:=640 height:=480
```

5. 공 추적
```bash
$ cd ~/soccer_ws
$ source devel.setup.bash
$ roslaunch ball_tracker_pkg ball_track.launch
```

6. 벽 감지
```bash
$ cd ~/soccer_ws
$ source devel/setup.bash
$ rosrun d435_pkg avoidWall.py
```

---
### 10/15 update

** avoid wall + ball tracking 확인 **

##### 실행 순서

1. 기본 구동 코드 (로봇 부팅 시 자동으로 켜져있음)
```bash
$ cd ~/sim2real_master
$ source install/setup.bash
$ roslaunch sim2real_master joy_control.launch
```
또는 mini pi 바탕화면의 아이콘 클릭

2. twist mux 실행
```bash
$ cd ~/soccer_ws
$ source devel/setup.bash
$ roslaunch wego_twist_mux twist_mux.launch
```

3. avoid wall 실행 (realsense2_camera rs_camera 자동으로 켜지니 중복되지 않게 주의!)
```bash
$ cd ~/soccer_ws
$ source devel/setup.bash
$ roslaunch d435_pkg avoid_wall.launch
```

4. ball tracking 실행, discrete 버전 (usb_cam usb_cam_node 사용 시 해당 노드 켜야 함!)
```bash
$ cd ~/soccer_ws
$ source devel/setup.bash
$ roslaunch ball_tracking_pkg mux_discrete_ball_track.launch
```
