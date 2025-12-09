import rospy
import time
from sim2real_msg.msg import Joy

# rospy 초기화는 노드 실행 시점에 한 번만 필요합니다.
rospy.init_node("standup_sequence_node", anonymous=True)


class StandUp:
    def __init__(self):
        self.pub = rospy.Publisher("/joy_msg", Joy, queue_size=10)
        self.joy_msg = Joy()

        # [수정된 핵심 로직] 로봇 제어 노드(구독자)가 준비될 때까지 대기합니다.
        rospy.loginfo("로봇 제어 노드가 토픽을 구독할 때까지 대기 중...")

        timeout_seconds = 10  # 최대 대기 시간 (초)
        start_time = time.time()

        # '/joy_msg' 토픽의 구독자 수가 1개 이상이 될 때까지 대기
        while self.pub.get_num_connections() < 1:
            if rospy.is_shutdown():
                rospy.logwarn("ROS가 종료되어 대기를 중단합니다.")
                return  # 노드가 종료되면 __init__ 종료

            # 타임아웃 확인
            if time.time() - start_time > timeout_seconds:
                rospy.logerr(f"[{timeout_seconds}초 초과] 로봇 노드 연결 실패. 시퀀스를 중단합니다.")
                # 연결 실패 시 노드 종료 신호를 보냅니다.
                rospy.signal_shutdown("Robot node connection timeout.")
                return

            rospy.sleep(0.5)

        rospy.loginfo("로봇 제어 노드 연결 확인 완료. 시퀀스를 시작합니다.")

    def start(self):
        # __init__에서 타임아웃 등으로 이미 노드 종료 신호가 발생했는지 확인
        if rospy.is_shutdown():
            return

        rospy.loginfo("--- 스탠드업 시퀀스 시작 ---")

        # 1단계: 일어서기 명령 (standby = 1.0)
        # self.joy_msg.standby = 1.0
        # self.joy_msg.running_standby_switch = 0.0
        self.joy_msg.lt = -1.0
        self.joy_msg.rt = -1.0
        self.joy_msg.start = 1.0
        self.pub.publish(self.joy_msg)
        rospy.loginfo("1. 일어서기 명령 (standby=1.0) 발행")

        # 로봇이 일어서는 데 필요한 충분한 시간 대기 (3초)
        rospy.sleep(3.0)

        self.joy_msg.lt = 1.0
        self.joy_msg.rt = 1.0
        self.joy_msg.start = 0.0
        self.pub.publish(self.joy_msg)
        rospy.sleep(1.0)

        # 2단계: 걸을 수 있도록 잠금 해제 (standby = 0.0, running_standby_switch = 1.0)
        # self.joy_msg.standby = 0.0
        # self.joy_msg.running_standby_switch = 1.0
        self.joy_msg.lt = -1.0
        self.joy_msg.rt = -1.0
        self.joy_msg.lb = 1.0
        self.pub.publish(self.joy_msg)
        rospy.loginfo("2. 걷기 잠금 해제 명령 (standby=0.0, running_standby_switch=1.0) 발행")

        # 다음 명령을 보내기 전 잠시 대기 (1초)
        rospy.sleep(1.0)

        # 3단계: joy_msg 원복 (running_standby_switch = 0.0)
        # self.joy_msg.running_standby_switch = 0.0
        self.joy_msg.lt = 1.0
        self.joy_msg.rt = 1.0
        self.joy_msg.lb = 0.0
        self.joy_msg.start = 0.0
        self.pub.publish(self.joy_msg)
        rospy.loginfo("3. joy_msg 원복 명령 (running_standby_switch=0.0) 발행")

        # 마지막 메시지가 확실히 전달되도록 잠시 대기
        rospy.sleep(0.5)

        # --- 노드 종료 ---
        rospy.loginfo("--- 스탠드업 시퀀스 완료 및 노드 종료 ---")
        rospy.signal_shutdown("StandUp sequence complete.")


if __name__ == "__main__":
    try:
        standup_node = StandUp()
        # __init__에서 오류(타임아웃 등)로 인해 노드가 종료되지 않았을 때만 start() 실행
        if not rospy.is_shutdown():
            standup_node.start()
    except rospy.ROSInterruptException:
        pass
