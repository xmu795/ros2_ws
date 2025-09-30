ros2 launch rtabmap_launch rtabmap.launch.py \
    stereo:=true \
    rgbd_sync:=false \
    frame_id:='base_link' \
    left_image_topic:=/camera/d435i/infra1/image_rect_raw \
    right_image_topic:=/camera/d435i/infra2/image_rect_raw \
    left_camera_info_topic:=/camera/d435i/infra1/camera_info \
    right_camera_info_topic:=/camera/d435i/infra2/camera_info \
    approx_sync:=true \
    Vis/MaxFeatures:=3000 \
    params:=["'Vis/MinInliers': '12', "OdomF2M/GuessMotion:=true", "Vis/UseIMU:=true", "GFTT/MinDistance": '5'] \
    rtabmap_viz:=false \
    subscribe_scan:=true \
    scan_topic:=/scan \
    imu_topic:=/imu/data \
    rviz:=false \
    subscribe_imu:=true \
    use_sim_time:=false \
    qos_image:=1 \
    qos_camera_info:=1 \
    qos_imu:=1 \
    qos_odom:=1 \
    qos_odom_info:=1 \
    qos_scan:=1 \
    Odom/Strategy:=1 \
    Odom/GuessMotion:=true \
    Vis/CorType:=1
