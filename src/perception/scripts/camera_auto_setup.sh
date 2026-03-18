#!/bin/bash

# 1. 장치 이름으로 현재 할당된 /dev/video 번호 자동 추출
HW40_DEV=$(v4l2-ctl --list-devices | grep -A 1 "HW40 webcam" | grep "/dev/video" | head -n 1 | xargs)
USB2_DEV=$(v4l2-ctl --list-devices | grep -A 1 "USB 2.0 Camera" | grep "/dev/video" | head -n 1 | xargs)

# 2. 감지된 카메라에 맞춰 하드웨어 파라미터 강제 설정
if [ ! -z "$HW40_DEV" ]; then
    echo "Restoring HW40 Webcam to Auto Mode Defaults on $HW40_DEV..."
    
    # [HW40 오토 모드 복원 - 카메라가 스스로 판단하게 합니다]
    v4l2-ctl -d $HW40_DEV --set-ctrl=auto_exposure=3                 # 3: Aperture Priority (자동 노출)
    v4l2-ctl -d $HW40_DEV --set-ctrl=white_balance_automatic=1       # 1: 자동 화이트밸런스 켬
    v4l2-ctl -d $HW40_DEV --set-ctrl=focus_automatic_continuous=1    # 1: 자동 초점 켬
    
    # [기본 화질 설정값 복구]
    v4l2-ctl -d $HW40_DEV --set-ctrl=brightness=0                    
    v4l2-ctl -d $HW40_DEV --set-ctrl=contrast=0                      
    v4l2-ctl -d $HW40_DEV --set-ctrl=saturation=64                   
    v4l2-ctl -d $HW40_DEV --set-ctrl=power_line_frequency=2          # 2: 60Hz (실내 조명 깜빡임 방지)

elif [ ! -z "$USB2_DEV" ]; then
    echo "Configuring USB 2.0 Camera on $USB2_DEV..."
    v4l2-ctl -d $USB2_DEV --set-ctrl=auto_exposure=1
    v4l2-ctl -d $USB2_DEV --set-ctrl=exposure_time_absolute=200     
    v4l2-ctl -d $USB2_DEV --set-ctrl=white_balance_automatic=0
    v4l2-ctl -d $USB2_DEV --set-ctrl=white_balance_temperature=5800 
    v4l2-ctl -d $USB2_DEV --set-ctrl=brightness=0                   
    v4l2-ctl -d $USB2_DEV --set-ctrl=contrast=70                    
    v4l2-ctl -d $USB2_DEV --set-ctrl=power_line_frequency=2
fi
