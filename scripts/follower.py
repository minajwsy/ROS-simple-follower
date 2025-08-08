#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
물체 추종 제어 시스템 - Follower 노드

이 파일은 LIMO 자율주행차가 LaserTracker에서 감지된 물체를 
부드럽고 안정적으로 추종하도록 제어하는 핵심 모듈입니다.

작성자: chutter@uos.de
용도: ROS-simple-follower 프로젝트 - PID 기반 물체 추종 제어
"""

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from simple_follower.msg import position as PositionMsg
from std_msgs.msg import String as StringMsg

class Follower:
    """
    물체 추종 제어 클래스
    
    주요 기능:
    1. LaserTracker로부터 물체 위치 정보 수신
    2. PID 제어기를 이용한 부드러운 추종 제어
    3. 안전한 후진 동작 및 속도 제한
    4. 로봇 움직임 명령 발행
    """
    
    def __init__(self):
        """
        Follower 초기화
        - 제어 파라미터 설정 및 토픽 구독/발행 설정
        """
        
        # ===== 제어 파라미터 로드 =====
        # switchMode: True면 버튼 한 번 누르면 on/off 토글, False면 버튼 누르고 있는 동안만 동작
        self.switchMode = rospy.get_param('~switchMode')
        
        # 로봇의 최대 이동 속도 제한 (m/s)
        self.max_speed = rospy.get_param('~maxSpeed')
        
        # 제어용 버튼 인덱스 (조이스틱 사용시)
        self.controllButtonIndex = rospy.get_param('~controllButtonIndex')

        # ===== 제어 상태 변수 =====
        self.buttonCallbackBusy = False  # 버튼 처리 중인지 확인
        self.active = False  # 추종 기능 활성화 여부

        # ===== ROS 토픽 발행 설정 =====
        # 로봇 제어 명령을 발행하는 퍼블리셔
        # Twist 메시지: 선속도(linear)와 각속도(angular) 정보 포함
        self.cmdVelPublisher = rospy.Publisher('/cmd_vel/yolo', Twist, queue_size=3)
        
        # ===== 조이스틱 제어 (현재 비활성화) =====
        # PS3 컨트롤러 등을 사용한 수동 제어 기능
        # self.joySubscriber = rospy.Subscriber('joy', Joy, self.buttonCallback)

        # ===== ROS 토픽 구독 설정 =====
        # LaserTracker에서 발행하는 물체 위치 정보를 구독
        self.positionSubscriber = rospy.Subscriber('/object_tracker/current_position', PositionMsg, self.positionUpdateCallback)
        
        # LaserTracker에서 발행하는 상태 정보를 구독 (예: "물체를 찾을 수 없음")
        self.trackerInfoSubscriber = rospy.Subscriber('/object_tracker/info', StringMsg, self.trackerInfoCallback)

        # ===== PID 제어기 초기화 =====
        # 목표 거리 설정 (물체와 유지하고 싶은 거리)
        targetDist = rospy.get_param('~targetDist')
        
        # PID 파라미터 로드 (YAML 파일에서)
        PID_param = rospy.get_param('~PID_controller')
        
        # PID 제어기 생성
        # 목표: [각도 0도(정면), 설정된 거리]
        # PID 파라미터: P(비례), I(적분), D(미분) 이득값
        self.PID_controller = simplePID([0, targetDist], PID_param['P'], PID_param['I'], PID_param['D'])

        # ===== 종료 처리 설정 =====
        # Ctrl+C로 프로그램 종료시 안전하게 로봇 정지
        rospy.on_shutdown(self.controllerLoss)

    def trackerInfoCallback(self, info):
        """
        LaserTracker 상태 정보 처리
        
        Args:
            info (String): LaserTracker에서 보내는 상태 메시지
                          예: "laser:nothing found" (물체를 찾을 수 없음)
        """
        # 현재는 단순히 경고 메시지만 출력
        # 필요시 물체를 잃어버렸을 때의 특별한 동작을 여기에 추가 가능
        rospy.logwarn("추적 정보: {}".format(info.data))
    
    def positionUpdateCallback(self, position):
        """
        물체 위치 업데이트 및 로봇 제어
        
        이 함수는 LaserTracker에서 새로운 물체 위치 정보를 받을 때마다 호출되며,
        PID 제어를 통해 로봇의 움직임을 계산하고 명령을 발행합니다.
        
        Args:
            position (PositionMsg): 물체의 위치 정보
                                   - angleX: 물체의 각도 (라디안, 0=정면)
                                   - distance: 물체까지의 거리 (미터)
        """
        
        # ===== 1단계: 위치 정보 추출 =====
        angleX = position.angleX      # 물체의 각도 (라디안)
        distance = position.distance  # 물체까지의 거리 (미터)

        # 현재 상태 로깅
        rospy.loginfo('물체 각도: {:.3f}rad, 거리: {:.3f}m, 목표 거리: {:.3f}m'.format(
            angleX, distance, self.PID_controller.setPoint[1]))
        
        # ===== 2단계: PID 제어 계산 =====
        # PID 제어기에 현재 상태 [각도, 거리]를 입력하고 제어 신호 받기
        # 출력: [각속도 제어신호, 선속도 제어신호]
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
        
        # ===== 3단계: 오차 계산 (디버깅용) =====
        distance_error = self.PID_controller.setPoint[1] - distance
        # distance_error > 0: 물체가 너무 가까움 → 로봇이 후진해야 함
        # distance_error < 0: 물체가 너무 멀음 → 로봇이 전진해야 함
        
        # ===== 4단계: 속도 제한 적용 =====
        # PID 출력을 최대 속도 범위 내로 제한
        # 음수 부호: PID 출력과 실제 로봇 움직임 방향을 맞춤
        angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
        linearSpeed = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
        
        # ===== 5단계: 안전한 후진 동작 보장 =====
        # 물체가 목표 거리의 80% 이내로 가까울 때
        if distance < self.PID_controller.setPoint[1] * 0.8:
            # 후진 속도가 너무 느리면 최소 후진 속도 보장
            # 이렇게 하면 물체가 가까워도 부드럽게 후진할 수 있음
            if linearSpeed > -0.1:
                linearSpeed = -0.15
                
        # ===== 6단계: 로봇 제어 메시지 생성 =====
        velocity = Twist()
        # 선속도 설정 (X축 방향만 사용, Y,Z는 0)
        velocity.linear = Vector3(linearSpeed, 0, 0.)
        # 각속도 설정 (Z축 회전만 사용, X,Y는 0)
        velocity.angular = Vector3(0., 0., angularSpeed)
        
        # ===== 7단계: 상세 로깅 =====
        # 현재 동작 상태 표시
        action = "전진" if linearSpeed > 0 else "후진" if linearSpeed < 0 else "정지"
        rospy.loginfo('거리 오차: {:.3f}m, 선속도: {:.3f}m/s ({}), 각속도: {:.3f}rad/s'.format(
            distance_error, linearSpeed, action, angularSpeed))
        
        # ===== 8단계: 제어 명령 발행 =====
        self.cmdVelPublisher.publish(velocity)

    def buttonCallback(self, joy_data):
        """
        조이스틱 버튼 입력 처리 (현재 사용 안 함)
        
        PS3 컨트롤러 등을 사용한 수동 제어시 사용하는 함수
        """
        # 연결 확인용 타이머 재설정
        self.controllerLossTimer.cancel()
        self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
        self.controllerLossTimer.start()

        # 중복 버튼 입력 방지
        if self.buttonCallbackBusy:
            return 
        else:
            # 별도 스레드에서 버튼 처리
            thread.start_new_thread(self.threadedButtonCallback, (joy_data, ))

    def threadedButtonCallback(self, joy_data):
        """
        조이스틱 버튼 처리 스레드 함수
        """
        self.buttonCallbackBusy = True

        if(joy_data.buttons[self.controllButtonIndex] == self.switchMode and self.active):
            # 추종 기능 비활성화
            rospy.loginfo('추종 기능을 중지합니다')
            self.stopMoving()
            self.active = False
            rospy.sleep(0.5)
        elif(joy_data.buttons[self.controllButtonIndex] == True and not(self.active)):
            # 추종 기능 활성화
            rospy.loginfo('추종 기능을 시작합니다')
            self.active = True
            rospy.sleep(0.5)

        self.buttonCallbackBusy = False

    def stopMoving(self):
        """
        로봇 즉시 정지
        
        모든 속도를 0으로 설정하여 로봇을 안전하게 정지시킵니다.
        """
        velocity = Twist()
        velocity.linear = Vector3(0., 0., 0.)   # 선속도 0
        velocity.angular = Vector3(0., 0., 0.)  # 각속도 0
        self.cmdVelPublisher.publish(velocity)

    def controllerLoss(self):
        """
        연결 손실 또는 비상 정지 처리
        
        조이스틱 연결이 끊어지거나 프로그램 종료시 호출되어
        로봇을 안전하게 정지시킵니다.
        """
        self.stopMoving()
        self.active = False
        rospy.loginfo('연결이 끊어졌습니다. 로봇을 정지합니다.')


class simplePID:
    """
    간단한 이산 시간 PID 제어기
    
    각도와 거리를 동시에 제어할 수 있는 다변수 PID 제어기입니다.
    """
    
    def __init__(self, target, P, I, D):
        """
        PID 제어기 초기화
        
        Args:
            target: 목표값 배열 [목표각도, 목표거리]
            P: 비례 이득 배열 [각도_P, 거리_P]
            I: 적분 이득 배열 [각도_I, 거리_I]  
            D: 미분 이득 배열 [각도_D, 거리_D]
        
        PID 제어 원리:
        - P (비례): 현재 오차에 비례하여 제어 → 빠른 응답
        - I (적분): 누적 오차에 비례하여 제어 → 정상상태 오차 제거
        - D (미분): 오차 변화율에 비례하여 제어 → 오버슈트 방지
        """

        # ===== 파라미터 크기 호환성 검사 =====
        if(not(np.size(P)==np.size(I)==np.size(D)) or 
           ((np.size(target)==1) and np.size(P)!=1) or 
           (np.size(target)!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('PID 파라미터 크기가 호환되지 않습니다')
        
        rospy.loginfo('PID 제어기 초기화 - P:{}, I:{}, D:{}'.format(P, I, D))
        
        # ===== PID 파라미터 저장 =====
        self.Kp = np.array(P)         # 비례 이득
        self.Ki = np.array(I)         # 적분 이득  
        self.Kd = np.array(D)         # 미분 이득
        self.setPoint = np.array(target)  # 목표값 [0도, 목표거리]
        
        # ===== 제어기 상태 변수 초기화 =====
        self.last_error = 0           # 이전 오차 (미분 계산용)
        self.integrator = 0           # 오차 누적값 (적분 계산용)
        self.integrator_max = float('inf')  # 적분 포화 방지 (현재 무제한)
        self.timeOfLastCall = None    # 이전 호출 시간 (시간 간격 계산용)
        
    def update(self, current_value):
        """
        PID 제어기 업데이트
        
        Args:
            current_value: 현재 측정값 [현재각도, 현재거리]
            
        Returns:
            제어 신호 [각속도_명령, 선속도_명령]
            
        PID 제어 수식:
        출력 = Kp*오차 + Ki*누적오차 + Kd*오차변화율
        """
        
        current_value = np.array(current_value)
        
        # ===== 입력 크기 검사 =====
        if(np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('현재값과 목표값의 크기가 다릅니다')
        
        # ===== 첫 번째 호출 처리 =====
        if(self.timeOfLastCall is None):
            # 첫 번째 호출시에는 시간 간격을 알 수 없으므로 0 출력
            self.timeOfLastCall = time.clock()
            return np.zeros(np.size(current_value))

        # ===== 오차 및 시간 계산 =====
        error = self.setPoint - current_value  # 오차 = 목표 - 현재
        P = error                              # 비례항 = 현재 오차
        
        currentTime = time.clock()
        deltaT = (currentTime - self.timeOfLastCall)  # 시간 간격

        # ===== 적분항 계산 =====
        # 적분 = 이전 적분 + (현재 오차 × 시간 간격)
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        
        # ===== 미분항 계산 =====  
        # 미분 = (현재 오차 - 이전 오차) / 시간 간격
        D = (error - self.last_error) / deltaT
        
        # ===== 다음 번 계산을 위한 값 저장 =====
        self.last_error = error
        self.timeOfLastCall = currentTime
        
        # ===== PID 제어 신호 계산 및 반환 =====
        control_signal = self.Kp*P + self.Ki*I + self.Kd*D
        return control_signal


if __name__ == '__main__':
    """
    메인 실행 부분
    - ROS 노드 초기화 및 실행
    """
    print('Follower 노드를 시작합니다...')
    
    # ROS 노드 초기화
    rospy.init_node('follower')
    
    # Follower 객체 생성
    follower = Follower()
    
    print('Follower가 정상적으로 시작되었습니다.')
    print('물체 위치 정보를 기다리는 중...')
    
    try:
        # ROS 메시지 수신 대기 (무한 루프)
        # LaserTracker에서 위치 정보가 들어올 때마다 positionUpdateCallback() 함수가 자동 호출됨
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Follower 노드가 종료되었습니다.')