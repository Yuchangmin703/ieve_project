#!/bin/bash

# 1. 장치 이름으로 현재 할당된 /dev/video 번호 자동 추출
HW40_DEV=$(v4l2-ctl --list-devices | grep -A 1 "HW40" | grep "/dev/video" | head -n 1 | xargs)
USB2_DEV=$(v4l2-ctl --list-devices | grep -A 1 "USB 2.0 Camera" | grep "/dev/video" | head -n 1 | xargs)

# 2. 감지된 카메라에 맞춰 하드웨어 파라미터 강제 설정
if [ ! -z "$HW40_DEV" ]; then
    echo "Configuring HW40 Webcam on $HW40_DEV (Factory Default Mode)..."
    
    # [HW40 기본 노출 및 초점]
    v4l2-ctl -d $HW40_DEV --set-ctrl=auto_exposure=3               # 기본값: 3 (Aperture Priority Mode / 자동)
    v4l2-ctl -d $HW40_DEV --set-ctrl=exposure_time_absolute=166    # 기본값: 166 (자동 모드일 땐 사실상 무시됨)
    v4l2-ctl -d $HW40_DEV --set-ctrl=focus_automatic_continuous=0  # 기본값: 0 (자동 초점 꺼짐)
    
    # [HW40 기본 색상 및 화이트밸런스]
    v4l2-ctl -d $HW40_DEV --set-ctrl=white_balance_automatic=1     # 기본값: 1 (자동 화이트밸런스 켜짐)
    v4l2-ctl -d $HW40_DEV --set-ctrl=white_balance_temperature=4600 # 기본값: 4600 (자동일 땐 무시됨)
    v4l2-ctl -d $HW40_DEV --set-ctrl=saturation=64                 # 기본값: 64
    
    # [HW40 기본 화질 설정]
    v4l2-ctl -d $HW40_DEV --set-ctrl=brightness=0                  # 기본값: 0
    v4l2-ctl -d $HW40_DEV --set-ctrl=contrast=0                    # 기본값: 0
    v4l2-ctl -d $HW40_DEV --set-ctrl=gamma=100                     # 기본값: 100
    v4l2-ctl -d $HW40_DEV --set-ctrl=sharpness=2                   # 기본값: 2
    
    # 전력 주파수 기본값은 1(50Hz)이지만, 한국(60Hz) 환경에서 화면 깜빡임이 생기면 이 부분만 2로 바꾸시면 됩니다.
    v4l2-ctl -d $HW40_DEV --set-ctrl=power_line_frequency=1        # 기본값: 1

elif [ ! -z "$USB2_DEV" ]; then
    echo "Configuring USB 2.0 Camera on $USB2_DEV..."
    # (USB 2.0 카메라는 기존 설정 유지)
    v4l2-ctl -d $USB2_DEV --set-ctrl=auto_exposure=1
    v4l2-ctl -d $USB2_DEV --set-ctrl=exposure_time_absolute=200     
    v4l2-ctl -d $USB2_DEV --set-ctrl=white_balance_automatic=1
    v4l2-ctl -d $USB2_DEV --set-ctrl=brightness=10                    
    v4l2-ctl -d $USB2_DEV --set-ctrl=contrast=80                    
    v4l2-ctl -d $USB2_DEV --set-ctrl=gamma=150
    v4l2-ctl -d $USB2_DEV --set-ctrl=power_line_frequency=2
    v4l2-ctl -d $USB2_DEV --set-ctrl=saturation=200
fi

echo "Camera Configuration Applied Successfully."
