# Mini Pi manual

### Ip 확인
mini pi를 와이파이에 연결한 후, ip를 확인하기 위해 번거롭게 터미널을 열 필요가 없습니다.

mini pi 뒷면의 LCD 화면 옆 스위치를 조종하여 다양한 상태 화면을 확인할 수 있습니다.
그 중, Robot IP 화면에서 로봇의 ip를 확인할 수 있습니다.

<img src="./startup/image/LCD_4.jpg" alt="ip 화면" width="90%" style="display: block; margin: 0 auto;">

<br>

만약 터미널로 확인할 경우, 다음 명령어를 입력해주세요.
```bash
hightorque@lubancat:~$ ifconfig

# ...(중략)

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.48  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::41fc:423d:be0a:b04  prefixlen 64  scopeid 0x20<link>
        ether dc:4a:9e:36:8b:1f  txqueuelen 1000  (Ethernet)
        RX packets 5942  bytes 1537953 (1.5 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 3228  bytes 2735445 (2.7 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

wlan0의 inet을 이용하여 원격접속에 사용할 수 있습니다

### 초기 와이파이 연결
처음 로봇을 개봉하여 부팅 후, 약 1분 간 기다리면 로봇과 블루투스 연결을 할 수 있습니다.

먼저, *nRF connection* 과 같은 블루투스 연결 및 조작 앱을 다운받아 주세요.

이후, Linux_SETUP_GATT와 같은 이름의 장치가 나타나면 연결해주세요.
<img src="./image/nRF_scanner.PNG" alt="nRF scanner 메인" width="25%" style="display: block; margin: 0 auto;">

연결후, 다음과 같은 창이 나타납니다. 표시된 것과 같이 0002, 0003으로 끝나는 uuid에서 각각 wifi 이름과 비밀번호를 입력합니다.
<img src="./image/nRF_scanner_connect.PNG" alt="nRF scanner 메인" width="25%" style="display: block; margin: 0 auto;">

0002로 끝나는 uuid에 연결하고자하는 와이파이 이름을 입력하고 전송하세요. utf8 형식, commend 형식으로 전송합니다.
<img src="./image/nRF_scanner_wifiname.PNG" alt="nRF scanner 메인" width="25%" style="display: block; margin: 0 auto;">

0003로 끝나는 uuid에 연결하고자하는 와이파이 비밀번호를 입력하고 전송하세요. utf8 형식, commend 형식으로 전송합니다.
<img src="./image/nRF_scanner_wifipw.PNG" alt="nRF scanner 메인" width="25%" style="display: block; margin: 0 auto;">

조금 기다리고 LCD 패널의 스위치를 옆으로 켜면 맨 위의 사진과 같이 wlan0으로 ip 주소가 나타납니다.

이후로는 해당 네트워크가 있으면 자동으로 연결됩니다.

네트워크를 변경하고 싶은 경우, 아래 gui연결을 참고해주세요

### gui 연결
gui 사용과 관련된 상세한 설명은 (wego minipi textbook)에 있으며, 해당 페이지에서는 gui를 통한 와이파이 변경 방법을 안내해드립니다.

네트워크가 연결되고, 로봇이 제자리에서 일어섰다면 gui에 접속할 수 있습니다.
LCD 패널에 표시된 주소로 http://$ROS_IP:8089 접속하면 아래와 같은 화면이 나타납니다. 네트워크가 연결된 상태에서 다른 네트워크로 변경하고 싶다면, gui의 'wifi 연결'페이지에 접속하세요.

<img src="./image/gui_wifi.png" alt="gui_wifi 페이지" width="100%" style="display: block; margin: 0 auto;">
<br>

와이파이 재조정 페이지에 접속하면, 아래와 같이 나타납니다. BLE 모드 시작을 누르고, 위의 초기 단계에서 블루투스 스캐너 앱을 통해 변경할 와이파이의 이름과 비밀번호를 입력하시면 됩니다. 

<img src="./image/wifi_page.png" alt="wifi 페이지" width="100%" style="display: block; margin: 0 auto;">
<br>

사용 후, 종료 버튼을 눌러주세요.