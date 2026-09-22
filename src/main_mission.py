"""Class Work 8 - SLAM: Explore the Unknown World.

Entry point that actually connects to the robot and runs the full mission:
  1. connect
  2. start logging all sensors (position / attitude / imu / esc / ToF / IR)
  3. run frontier-DFS exploration + mapping (unknown map, self-localized via
     odometry, walls sensed with ToF + Sharp IR)
  4. stop logging
  5. print + save the Start/End position report the assignment asks for

This file did not exist before - explore_and_map_all() in chassis.py was
never actually wired up to a robot connection, so the mission was never run
end-to-end. Run this on Monday during the test.
"""

import sys
import os

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

    try:
        report = chassis.explore_and_map_all()
    finally:
        chassis.stop_sensors()
        ep_robot.close()

    print("\nMission finished.")
    print(f"Start: {report['start_grid']}  End: {report['end_grid']}")
    print("Now run analysis/generate_map_report.py to build the Map, Trajectory,")
    print("and Accuracy/Coverage numbers for submission.")
    return report


if __name__ == "__main__":
    main()
