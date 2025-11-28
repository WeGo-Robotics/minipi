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