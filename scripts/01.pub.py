#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 노드(Node)의 예제입니다.
# 이 예제를 통해 학생들은 ROS 노드를 사용하여 메시지를 발행하고 주기적으로 업데이트하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(std_msgs.msg)을 가져옵니다.
import rospy
from std_msgs.msg import Int32

# ROS 노드를 초기화합니다. 노드 이름은 "wego_pub_node"로 지정됩니다.
rospy.init_node("wego_pub_node")  # ROS 1단계(필수): 노드 이름 정의

# ROS 퍼블리셔(Publisher)를 설정합니다.
# "/counter" 토픽에 Int32 메시지를 발행하는 퍼블리셔를 만듭니다.
# "queue_size"는 메시지 대기열 크기를 나타냅니다.
pub = rospy.Publisher("/counter", Int32, queue_size=1)  # ROS 2단계(필수): 퍼블리셔 설정

# 메시지 발행 주기를 설정합니다. 이 예제에서는 10Hz로 설정합니다.
rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 발행 주기 설정

# Int32 메시지 타입의 메시지 객체를 생성합니다.
msg = Int32()  # 메시지 타입 설정 및 초기화

# ROS가 종료될 때까지 아래의 코드를 반복 실행합니다.
while not rospy.is_shutdown():
    # 퍼블리셔를 사용하여 Int32 메시지를 발행합니다.
    pub.publish(msg)  # ROS 3단계(필수): 메시지 발행

    # 메시지 객체의 데이터 값을 1씩 증가시킵니다.
    msg.data += 1  # 메시지 항목(데이터) - 데이터 값 변경

    # 현재 메시지와 메시지 데이터를 출력합니다.
    print(f"{msg}")  # 출력: 메시지
    print(f"{msg.data}")  # 출력: 메시지 항목(데이터)

    # 지정한 발행 주기에 따라 슬립합니다.
    rate.sleep()  # ROS 3-1단계(옵션): 발행 주기 실행