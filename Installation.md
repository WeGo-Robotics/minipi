# Mini pi 사용자 설치 가이드

이 설치 가이드는 mini pi를 처음 사용하시는 분들을 위한 기본 예제 워크스페이스 설치 가이드입니다.

vscode extension에서 **Runme Notebooks for DevOps**를 다운받아주세요. 
이후 md 파일에서 코드를 실행하며 설치를 진행할 수 있습니다.

mini pi 로봇 내부에서 git clone을 받습니다.

만약 해당 워크스페이스를 새로 다운받는 경우에는 우선 아래 명령어를 따라주세요.

```sh
cd
rm -rf wego_minipi_ws .config/autostart/* startup
rm -rf /Desktop/joy_Switch_alg.destop /Desktop/robot_wego.desktop /Desktop/custom_startup.desktop
```

이후 아래 명령어를 통해 레포지토리를 다운받고, merge_manual 브랜치로 전환합니다.

```sh
git clone https://github.com/WeGo-Robotics/minipi.git
cd minipi
git checkout merge_manual
```

```sh
sudo apt  update
sudo apt  install  software-properties-common
sudo add-apt-repository  ppa:deadsnakes/ppa
sudo apt  update
sudo apt  install  python3.10  python3.10-distutils  python3.10-venv
```

만약 위의 명령어로 설치가 되지 않는다면 아래 명령어를 통해 설치해주세요

```sh
sudo apt update
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget

wget https://www.python.org/ftp/python/3.10.13/Python-3.10.13.tgz
tar -xf Python-3.10.13.tgz

cd Python-3.10.13
./configure --enable-optimizations --enable-shared
make -j$(nproc)
sudo make altinstall

echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/python3.10.conf
sudo ldconfig
```

```sh
python3 --version
# Python 3.8.x

python3.10 --version
# Python 3.10.x
```

```sh
cd ~/minipi/startup
chmod +x *
./setting.sh
./update_bashrc.sh
./install_pkg_deps.sh
./install_llm.sh
./install_llm_model.sh
```

```sh
cd ~/wego_minipi_ws
pip install -r requirements.txt
```

```sh
cd ~/wego_minipi_ws/src/sim2real_msg/msg
find ~/sim2real_master/install/share/sim2real_msg/msg -maxdepth 1 -name "*.msg" | grep -v 'lowlevel_' | xargs -I {} cp {} .
sudo vi ../CMakeLists.txt
```

CMakeLists.txt에 추가된 메세지를 넣습니다.

```sh
add_message_files(
  FILES
  Yolo.msg
  YoloDetect.msg
  # 추가된 .msg 파일
)
```

이후, 빌드를 수행합니다.
만약 기존에 사용하던 워크스페이스가 있다면 편의에 따라 패키지를 통합하거나 독립적으로 사용하시면 됩니다.

```sh
cd ~/wego_minpi_ws
catkin_make
```

이후, 로봇을 재부팅합니다.

wego_minipi_ws에서 아래 명령어를 실행하면 내부에 설치된 것을 확인할 수 있습니다.

```sh
# 축구 공 추적 launch
roslaunch socccer_demo ball_tracking_demo.launch
```

```sh
# 공 -> 골대 launch (환경에 따른 yolov11 모델 변경 필요)
roslaunch soccer_demo ball_goal_demo.launch
```