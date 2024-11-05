#!/bin/bash

# LaserScanメッセージを生成する関数
publish_scan() {
    local ranges=$(printf "%.1f," $(yes 1.0 | head -n 360))
    local intensities=$(printf "%.1f," $(yes 1000.0 | head -n 360))

    # 最後のコンマを削除
    ranges=${ranges%?}
    intensities=${intensities%?}

    while true; do
        ros2 topic pub /scan sensor_msgs/msg/LaserScan "{
            header: {stamp: now, frame_id: 'laser_link'},
            angle_min: -3.14,
            angle_max: 3.14,
            angle_increment: 0.0174533,
            time_increment: 0.0,
            scan_time: 0.05,
            range_min: 0.12,
            range_max: 3.5,
            ranges: [$ranges],
            intensities: [$intensities]
        }" -r 20
        sleep 0.05
    done
}

# LaserScanデータをPublish
publish_scan

