# Mini Pi 축구 데모 패키지
휴머노이드 로봇의 축구 동작을 위한 패키지 모음
d435(rgb+depth), 광각 카메라, rockchip npu 등의 툴을 사용한 환경에서 초기 개발이 이루어짐
향후 다른 휴머노이드 로봇에서 활용할 수 있도록 정리하고 있음
로봇의 msg 타입 등을 고려하여 수정할 부분을 수정하면 됨 (이 부분에서 변경을 최소화하도록 하는 것이 목표)

향후 로봇이 자체 web rtc를 하여 사용자는 핸드폰 앱(또는 웹 서버)으로 노드를 실행할 수 있도록(복잡한 환경에서 지연 없이...)

#### ball_tracker_pkg
공 추적 및 골대-공 맞춤 기능

cam_setting.launch에서 카메라들 밝기, 노출 등 설정 변환 및 저장, 불러오기 기능 (변경한 상태로 끄면 로봇 종료, usb 재 체결 전까지 유지됨)

---
#### d435_pkg
벽, 장애물 인식 및 회피 기능
realsense2_camera 라이브러리 설치 필수


---
#### dataset_making
yolo 학습을 위한 데이터셋 만들기 기능
record_bag.launch로 bag 파일 저장, 이후 get_image.launch로 이미지 추출

---
#### ros1_commander

---
#### sim2real_msg (mini pi 전용 msg)
개발당시 사용한 로봇인 mini pi에서 사용하는 전용 msg
향후 다른 로봇에서 필수적으로 사용해야 하는 msg를 넣어서 쓰기 (devel, install 시 번거로움 방지)

---
#### using_llm
llm 사용 기능
rkllama 연결 및 gui 켜기

---
#### wego_twist_mux
/cmd_vel 충돌을 방지하기 위한 twist mux 정의 (ros twist_mux)
각 기능 별 우선순위를 정해놓으면 됨.

---
#### yolo11_detect_pkg
yolo11을 사용한 검출 기능 (mini pi에서는 rockchip을 사용하였으므로 rknn 형식 포맷 사용함)

---
#### web_gui_pkg
동일 네트워크 연결 시 웹/모바일로 로봇 구동 가능한 패키지



# 2025 로보월드 전용 GUI

  

![roboworld gui](/etc/docs/image/roboworld_gui.png)

  

아래 명령어를 통해 축구 관련 launch 실행, 로그 확인, 영상 확인이 가능함

  

```bash

$ cd  ~/soccer_ws

$ source  devel/setup.bash

$ python3  src/roboworld_soccer.py

```

  

아래 명령어를 통해 mini pi llm yolo, gui 실행 가능

  

```bash

$ cd  ~/soccer_ws

$ source  devel/setup.bash

$ roslaunch  using_llm  yolo_llm_pi.launch

```

```bash

$ cd  ~/soccer_ws

$ source  devel/setup.bash

$ roslaunch  using_llm  llm_gui.launch

```

  

![llm gui](/etc/docs/video/minipi_llm_gui.webm)

gui 내에서 원하는 패키지 런치파일 실행 및 정지, 간단한 로그 확인, 영상 토픽 확인 가능 (향후 이것을 로봇 내 web rtc 연결로 구현 예정)

---

#### AutoStartSetting 폴더 설명

hightorque사에서 로봇 출고 시 자동으로 구동 코드를 실행하도록 하였음 (settings > Session and Startup > joy_switch_alg, 새 sim2real 버전 출시 이후는 정확하지 않음)

이는 로그인 시점에서 자동으로 실행되게 하는 것인데, 새로 만든 gui 전용 startup(rosbridge+gui.py)을 실행하게 하면 충돌로 인해 잘 되지 않음

따라서 새로운 스크립트 파일을 통해 로봇 구동 > rosbridge > gui.py 를 실행하도록 설정하여 Startup에 지정함 => AutoStartSetting/master_autostart.sh

NiceGUI-ROS.desktop: gui 설정되도록 하는 아이콘. 바탕화면에 두고 쓰면 됨
soccer_web_gui.sh: 홈 디렉토리에 설치, 웹 gui 가 실행되도록 하는 스크립트 파일

