# 1) 개별 launch ↔ launch 충돌
LAUNCH_CONFLICTS = {
    ("sim2real_master", "joy_teleop.launch"): [("sim2real_master", "joy_teleop_kid.launch")],
    ("sim2real_master", "gdb_joy_control_hi.launch"): [
        ("sim2real_master", "gdb_joy_control_pi.launch"),
        ("sim2real_master", "gdb_joy_control_pi_plus.launch"),
        ("sim2real_master", "joy_control_hi.launch"),
        ("sim2real_master", "joy_control_pi.launch"),
        ("sim2real_master", "joy_control_pi_plus.launch"),
    ],
    ("sim2real_master", "gdb_joy_control_pi.launch"): [
        ("sim2real_master", "gdb_joy_control_hi.launch"),
        ("sim2real_master", "gdb_joy_control_pi_plus.launch"),
        ("sim2real_master", "joy_control_hi.launch"),
        ("sim2real_master", "joy_control_pi.launch"),
        ("sim2real_master", "joy_control_pi_plus.launch"),
    ],
    ("sim2real_master", "gdb_joy_control_pi_plus.launch"): [
        ("sim2real_master", "gdb_joy_control_hi.launch"),
        ("sim2real_master", "gdb_joy_control_pi.launch"),
        ("sim2real_master", "joy_control_hi.launch"),
        ("sim2real_master", "joy_control_pi.launch"),
        ("sim2real_master", "joy_control_pi_plus.launch"),
    ],
    ("sim2real_master", "joy_control_hi.launch"): [
        ("sim2real_master", "gdb_joy_control_hi.launch"),
        ("sim2real_master", "gdb_joy_control_pi.launch"),
        ("sim2real_master", "gdb_joy_control_pi_plus.launch"),
        ("sim2real_master", "joy_control_pi.launch"),
        ("sim2real_master", "joy_control_pi_plus.launch"),
    ],
    ("sim2real_master", "joy_control_pi.launch"): [
        ("sim2real_master", "gdb_joy_control_hi.launch"),
        ("sim2real_master", "gdb_joy_control_pi.launch"),
        ("sim2real_master", "gdb_joy_control_pi_plus.launch"),
        ("sim2real_master", "joy_control_hi.launch"),
        ("sim2real_master", "joy_control_pi_plus.launch"),
    ],
    ("sim2real_master", "joy_control_pi_plus.launch"): [
        ("sim2real_master", "gdb_joy_control_hi.launch"),
        ("sim2real_master", "gdb_joy_control_pi.launch"),
        ("sim2real_master", "gdb_joy_control_pi_plus.launch"),
        ("sim2real_master", "joy_control_hi.launch"),
        ("sim2real_master", "joy_control_pi.launch"),
    ],
}

# 2) 개별 launch ↔ 특정 패키지 전체 충돌
PACKAGE_CONFLICTS = {
    ("d435_pkg", "avoid_wall.launch"): ["realsense2_camera", "yolo11_detect_pkg"],
    ("yolo11_detect_pkg", "yolo_detect_efficient_d435.launch"): ["realsense2_camera"],
}

# 3) 특정 패키지 ↔ 특정 패키지 전체 충돌
PACKAGE_GROUP_CONFLICTS = {"sim2real_master": ["ball_tracker_pkg"], "soccer_demo": ["ball_tracker_pkg", "d435_pkg", "using_llm", "yolo11_detect_pkg"]}

# 4) 개별 launch는 해당 패키지에서 유일하게 동작
EXCLUSIVE_PACKAGES = ["ball_tracker_pkg", "soccer_demo", "wego_twist_mux"]

# 5) 항상 선택할 수 없는 launch
BLOCKED_LAUNCHES = [
    ("sim2real_master", "gdb_joy_control_hi.launch"),
    ("sim2real_master", "gdb_joy_control_pi.launch"),
    ("sim2real_master", "gdb_joy_control_pi_plus.launch"),
    ("sim2real_master", "joy_control_hi.launch"),
    ("sim2real_master", "joy_control_pi.launch"),
    ("sim2real_master", "joy_control_pi_plus.launch"),
    ("realsense2_camera", "demo_pointcloud.launch"),
    ("realsense2_camera", "rs_d435_camera_with_model.launch"),
    ("realsense2_camera", "demo_t265.launch"),
    ("realsense2_camera", "rs_from_file.launch"),
    ("realsense2_camera", "opensource_tracking.launch"),
    ("realsense2_camera", "rs_multiple_devices.launch"),
    ("realsense2_camera", "rs_aligned_depth.launch"),
    ("realsense2_camera", "rs_rgbd.launch"),
    ("realsense2_camera", "rs_rtabmap.launch"),
    ("realsense2_camera", "rs_d400_and_t265.launch"),
    ("realsense2_camera", "rs_t265.launch"),
]
