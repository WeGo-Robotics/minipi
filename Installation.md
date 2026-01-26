# Mini pi 사용자 설치 가이드

이 설치 가이드는 mini pi를 처음 사용하시는 분들을 위한 기본 예제 워크스페이스 설치 가이드입니다.

vscode extension에서 **Runme Notebooks for DevOps**를 다운받아주세요. 
이후 md 파일에서 코드를 실행하며 설치를 진행할 수 있습니다.

mini pi 로봇 내부에서 git clone을 받습니다.

만약 해당 워크스페이스를 새로 다운받는 경우에는 우선 아래 명령어를 따라주세요.

```sh {"name":"initial"}
cd
rm -rf wego_minipi_ws .config/autostart/* startup
rm -rf /Desktop/joy_Switch_alg.destop /Desktop/robot_wego.desktop /Desktop/custom_startup.desktop
./install_python310.sh
cd ~/minipi/startup
chmod +x *
./setting.sh
./update_bashrc.sh
./install_pkg_deps.sh
cd ~/wego_minipi_ws
pip install -r requirements.txt
cd ~/wego_minipi_ws/src/sim2real_msg/msg
find ~/sim2real_master/install/share/sim2real_msg/msg -maxdepth 1 -name "*.msg" | grep -v 'lowlevel_' | xargs -I {} cp {} .
cd ~/wego_minipi_ws
catkin_make
```

이후, 로봇을 재부팅합니다.

```sh
# 축구 공 추적 launch
roslaunch socccer_demo ball_tracking_demo.launch
```

```sh
# 공 -> 골대 launch (환경에 따른 yolov11 모델 변경 필요)
roslaunch soccer_demo ball_goal_demo.launch
```