짧게 결론부터:

**새로 조이스틱 코드를 만들 필요 없습니다.**
지금 있는 `joy_teleop + YAML` 그대로 쓰고, **twist_mux로 소스 분리/우선순위**만 주면 충돌이 끝나요.
GUI는 **비상정지(E-Stop) 버튼만 있는 초미니 패널** 정도면 충분합니다. (전체 조작을 GUI로 옮길 필요 X)

왜 이 선택?

* 5주 남음 → **가장 안정적이고 빠른** 방법
* 조이스틱/코드/GUI가 **각자 토픽 분리** → 충돌 제거
* **우선순위**로 “사람(조이스틱) > 자동(코드)” 보장
* **E-Stop**은 버튼 한 번에 즉시 정지(소프트 락) + 하드웨어 스위치 병행

---

## 딱 이것만 하면 됨 (4단계)

### 1) 토픽 분리 + 우선순위(twist_mux)

설치:

```bash
sudo apt-get install ros-noetic-twist-mux
```

`config/twist_mux.yaml`

```yaml
topics:
  - {name: joystick, topic: /cmd_vel/joy, timeout: 0.5, priority: 100}
  - {name: gui,      topic: /cmd_vel/gui, timeout: 0.5, priority: 70}
  - {name: auto,     topic: /cmd_vel/auto, timeout: 0.5, priority: 50}

locks:
  - {name: estop, topic: /twist_mux/lock, timeout: 0.0, priority: 255}
```

런치:

```xml
<node pkg="twist_mux" type="twist_mux" name="twist_mux" output="screen">
  <rosparam file="$(find your_pkg)/config/twist_mux.yaml" command="load"/>
</node>
```

> twist_mux 출력이 실제 `/cmd_vel`이 됩니다.

### 2) 조이스틱 YAML만 수정(코드 작성 불필요)

기존 `joy_teleop.yaml`에서 **출력 토픽만** `/cmd_vel/joy`로 변경:

```yaml
teleop:
  move:
    type: topic
    topic_name: "/cmd_vel/joy"          # ← 여기만 바꾸면 끝
    message_type: "geometry_msgs/Twist"
    deadman_buttons: [4]
    axis_mappings:
      - {axis: 1, target: "linear.x",  scale: 0.5}
      - {axis: 3, target: "angular.z", scale: 1.2}
```

### 3) 코드(usr2ctrl)는 `/cmd_vel/auto`로 발행

자동 주행/제자리걷기 등 코드 쪽 Twist는 **/cmd_vel/auto**로:

```python
# pub = rospy.Publisher('/cmd_vel', Twist, ...)  # X
pub = rospy.Publisher('/cmd_vel/auto', Twist, queue_size=10)  # O
```

“제자리걷기/정지” 같은 **모드 전환**은 서비스/전용 토픽으로 통합(joy와 코드가 같은 인터페이스 사용)하면 충돌이 사라집니다.

### 4) 비상정지(E-Stop)

* **조이스틱 버튼**으로 잠금:

```yaml
  estop_lock:
    type: topic
    topic_name: "/twist_mux/lock"
    message_type: "std_msgs/Bool"
    deadman_buttons: [1]  # 예: B 버튼
    message_value:
      - {target: "data", value: true}
```

* **해제 버튼**도 원하면 하나 더:

```yaml
  estop_unlock:
    type: topic
    topic_name: "/twist_mux/lock"
    message_type: "std_msgs/Bool"
    deadman_buttons: [2]  # 예: X 버튼
    message_value:
      - {target: "data", value: false}
```

* **급하면 CLI도 가능**:
  잠금: `rostopic pub /twist_mux/lock std_msgs/Bool "data: true"`
  해제: `rostopic pub /twist_mux/lock std_msgs/Bool "data: false"`

* **초미니 GUI(선택)**: 잠금/해제 두 버튼만 있는 간단 패널에서 `/twist_mux/lock` True/False 발행.
  (전체 텔레옵 GUI는 굳이 만들 필요 없고, E-Stop 버튼만 있어도 운영 편의성이 확 올라갑니다.)

---

## 요약

* **새 조이스틱 코드 X** → 기존 `joy_teleop.yaml`만 `/cmd_vel/joy`로 바꿈
* **twist_mux 도입** → `/cmd_vel/joy`, `/cmd_vel/auto`, (옵션 `/cmd_vel/gui`)를 **우선순위로 합성 → `/cmd_vel`**
* **E-Stop**: `/twist_mux/lock` (조이스틱 버튼 + GUI 버튼 + CLI 어떤 것도 OK)
* **모드 전환**은 서비스/전용 토픽으로 통합 → 충돌 원인 제거

이렇게 하면 “조이스틱 제어 유지 + 코드 병행 + 즉시 비상정지”가 동시에 됩니다.
