#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
라이다 추적 시스템 - LaserTracker 노드

이 파일은 LIMO 자율주행차의 라이다 센서 데이터를 처리하여
가장 가까운 물체의 위치를 감지하고 추적하는 핵심 모듈입니다.

작성자: chutter@uos.de
용도: ROS-simple-follower 프로젝트 - 라이다 기반 물체 추적
"""

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from simple_follower.msg import position

class laserTracker:
    """
    라이다 센서를 이용한 물체 추적 클래스
    
    주요 기능:
    1. 라이다 스캔 데이터 수신 및 처리
    2. 노이즈 필터링을 통한 안정적인 물체 감지
    3. 가장 가까운 물체의 위치 정보 발행
    """
    
    def __init__(self):
        """
        LaserTracker 초기화
        - 파라미터 설정 및 토픽 구독/발행 설정
        """
        # ===== 멤버 변수 초기화 =====
        self.lastScan = None  # 이전 스캔 데이터 저장 (노이즈 필터링용)
        
        # ===== 노이즈 필터링 파라미터 =====
        # winSize: 현재 측정점 주변에서 검사할 점의 개수 (좌우 각각)
        # 예: winSize=2이면 현재점 기준 좌우 2개씩, 총 5개 점을 검사
        self.winSize = rospy.get_param('~winSize')
        
        # deltaDist: 이전 스캔과 현재 스캔의 거리 차이 허용값 (미터)
        # 이 값보다 차이가 작으면 같은 물체로 인정 (노이즈가 아님)
        self.deltaDist = rospy.get_param('~deltaDist')
        
        # ===== ROS 토픽 구독 설정 =====
        # 라이다 센서 데이터를 받는 구독자
        # '/hokuyo_base/scan' 또는 '/scan' 토픽에서 LaserScan 메시지 수신
        self.scanSubscriber = rospy.Subscriber('/hokuyo_base/scan', LaserScan, self.registerScan)
        
        # ===== ROS 토픽 발행 설정 =====
        # 감지된 물체의 위치 정보를 발행하는 퍼블리셔
        # follower 노드가 이 정보를 받아서 로봇을 제어함
        self.positionPublisher = rospy.Publisher('/object_tracker/current_position', position, queue_size=3)
        
        # 추적 상태 정보를 발행하는 퍼블리셔 (디버깅용)
        self.infoPublisher = rospy.Publisher('/object_tracker/info', StringMsg, queue_size=3)

    def registerScan(self, scan_data):
        """
        라이다 스캔 데이터 처리 및 물체 위치 계산
        
        Args:
            scan_data (LaserScan): 라이다 센서에서 받은 스캔 데이터
            
        처리 과정:
        1. 스캔 데이터 전처리 (노이즈 제거)
        2. 가장 가까운 물체 후보 탐색
        3. 이전 스캔과 비교하여 노이즈 필터링
        4. 유효한 물체의 위치 정보 발행
        """
        
        # ===== 1단계: 라이다 데이터 전처리 =====
        # LaserScan의 ranges 배열을 numpy 배열로 변환
        # ranges[i]는 각도 i에서 측정된 거리값
        ranges = np.array(scan_data.ranges)
        
        # 너무 가까운 측정값 제거 (0.1m 미만)
        # 이런 값들은 보통 센서 노이즈이거나 로봇 자체 부품의 반사
        ranges[ranges < 0.1] = float('inf')
        
        # ===== 주석 처리된 코드: 시야각 제한 =====
        # 필요시 전방 90도만 사용하도록 제한 가능
        # num_ranges = len(ranges)
        # angles = scan_data.angle_min + np.arange(num_ranges) * scan_data.angle_increment
        # ranges[np.abs(angles) > np.pi/2.0] = float('inf')
        
        # ===== 2단계: 거리 기준 정렬 =====
        # 가까운 거리부터 차례로 검사하기 위해 인덱스를 거리 순으로 정렬
        # sortedIndices[0]이 가장 가까운 점의 인덱스
        sortedIndices = np.argsort(ranges)
        
        # ===== 3단계: 유효한 최소 거리 찾기 =====
        minDistanceID = None    # 가장 가까운 유효 물체의 인덱스
        minDistance = float('inf')  # 가장 가까운 유효 물체까지의 거리

        if(not(self.lastScan is None)):
            # 이전 스캔 데이터가 있는 경우 (첫 번째 스캔이 아님)
            # 노이즈 필터링을 위해 이전 스캔과 비교
            
            for i in sortedIndices:
                # 가까운 거리부터 순서대로 검사
                tempMinDistance = ranges[i]
                
                # ===== 4단계: 노이즈 검사 =====
                # 현재 측정점이 진짜 물체인지 노이즈인지 판단
                # 방법: 이전 스캔에서 비슷한 위치에 비슷한 거리의 측정값이 있었는지 확인
                
                # 검사할 윈도우 범위 계산 (현재점 ± winSize)
                # 배열 범위를 벗어나지 않도록 clipping
                windowIndex = np.clip([i-self.winSize, i+self.winSize+1], 0, len(self.lastScan))
                
                # 이전 스캔에서 해당 윈도우 영역의 거리값들 추출
                window = self.lastScan[windowIndex[0]:windowIndex[1]]

                with np.errstate(invalid='ignore'):
                    # 이전 스캔의 윈도우 영역에서 현재 측정값과 비슷한 거리가 있는지 확인
                    # abs(window-tempMinDistance) <= self.deltaDist
                    # 즉, 거리 차이가 deltaDist 이하인 점이 있는지 확인
                    if(np.any(abs(window-tempMinDistance) <= self.deltaDist)):
                        # 비슷한 거리의 측정값이 이전에도 있었음
                        # -> 이것은 실제 물체일 가능성이 높음 (노이즈가 아님)
                        
                        minDistanceID = i  # 해당 인덱스 저장
                        minDistance = ranges[minDistanceID]  # 해당 거리 저장
                        break  # 가장 가까운 유효한 물체를 찾았으므로 루프 종료
        
        # ===== 5단계: 다음 번 비교를 위해 현재 스캔 저장 =====
        self.lastScan = ranges
        
        # ===== 6단계: 결과 처리 및 발행 =====
        if(minDistance > scan_data.range_max):
            # 유효한 물체를 찾지 못한 경우
            # (모든 측정값이 센서 최대 범위를 벗어나거나 노이즈로 판정됨)
            
            # 경고 메시지 출력
            rospy.logwarn('laser no object found')
            # 추적 시스템에 물체 없음을 알림
            self.infoPublisher.publish(StringMsg('laser:nothing found'))
            
        else:
            # 유효한 물체를 찾은 경우
            
            # ===== 물체의 각도 계산 =====
            # 라이다 스캔에서 각도 계산 공식:
            # angle = angle_min + index * angle_increment
            # angle_min: 스캔 시작 각도, angle_increment: 각도 간격
            minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
            
            # ===== 위치 정보 발행 =====
            # position 메시지 생성 및 발행
            # (각도X, 각도Y, 거리) - 각도Y는 2D 라이다에서 사용 안 함(임의값 42)
            # 0도는 로봇의 정면을 의미
            self.positionPublisher.publish(position(minDistanceAngle, 42, minDistance))


if __name__ == '__main__':
    """
    메인 실행 부분
    - ROS 노드 초기화 및 실행
    """
    print('LaserTracker 노드를 시작합니다...')
    
    # ROS 노드 초기화
    rospy.init_node('laser_tracker')
    
    # LaserTracker 객체 생성
    tracker = laserTracker()
    
    print('LaserTracker가 정상적으로 시작되었습니다.')
    print('라이다 데이터를 기다리는 중...')
    
    try:
        # ROS 메시지 수신 대기 (무한 루프)
        # 라이다 데이터가 들어올 때마다 registerScan() 함수가 자동 호출됨
        rospy.spin()
    except rospy.ROSInterruptException:
        print('LaserTracker 노드가 종료되었습니다.')