"""Class Work 8 - SLAM: Explore the Unknown World.

Entry point that actually connects to the robot and runs the full mission:
  1. connect
  2. start logging all sensors (position / attitude / imu / esc / ToF)
  3. run frontier-DFS exploration + mapping (unknown map, self-localized via
     odometry, walls sensed with ToF + Gimbal scan)
  4. stop logging
  5. print + save the Start/End position report the assignment asks for
"""

import sys
import os
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robomaster import robot
from config_loader import load_config
from chassis import ChassisController


def main():
    config = load_config()
    connection_type = config.get("robot", {}).get("connection_type", "ap")

    ep_robot = robot.Robot()
    print(f"Connecting to RoboMaster using '{connection_type}' mode...")
    if connection_type == "ap":
        ep_robot.initialize(conn_type="ap", proto_type="udp")
    else:
        ep_robot.initialize(conn_type=connection_type)

    chassis = ChassisController(ep_robot, config)
    chassis.setup_csv_headers()
    chassis.start_sensors()

    report = None
    try:
        report = chassis.explore_and_map_all()
        
        # 🟢 ทำให้หน้าต่าง OpenCV Real-time ค้างไว้จนกว่าผู้ใช้จะกดปุ่ม 'q' หรือ Esc เพื่อดูผลลัพธ์สุดท้าย
        print("\n[+] ภารกิจสำรวจเสร็จสิ้น! กดปุ่ม 'q' หรือ Esc ที่หน้าต่างแผนที่ Grid เพื่อปิดโปรแกรม...")
        while True:
            key = cv2.waitKey(100) & 0xFF
            if key in (ord("q"), 27):  # กด q หรือ ESC
                break
    finally:
        chassis.stop_sensors()
        ep_robot.close()

    if report:
        print("\nMission finished.")
        print(f"Start: {report['start_grid']}   End: {report['end_grid']}")
        print("Now run analysis/generate_map_report.py to build the Map, Trajectory,")
        print("and Accuracy/Coverage numbers for submission.")
    return report


if __name__ == "__main__":
    main()
