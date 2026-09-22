import csv
import time
import os
import json
import threading
from datetime import datetime
import cv2
import numpy as np


class ChassisController:
    def __init__(self, ep_robot, config):
        self.ep_robot = ep_robot
        self.config = config
        self.ep_chassis = ep_robot.chassis
        self.ep_sensor = ep_robot.sensor
        self.ep_gimbal = ep_robot.gimbal

        self.current_tof_dist_mm = 9999
        self.current_yaw = 0.0

        self.data_dir = config["data_collection"]["data_dir"]
        self.buffer_time = config["data_collection"]["buffer_time"]

        self.freq_pos = config["data_collection"]["frequencies"]["position"]
        self.freq_att = config["data_collection"]["frequencies"]["attitude"]
        self.freq_imu = config["data_collection"]["frequencies"]["imu"]
        self.freq_esc = config["data_collection"]["frequencies"]["esc"]
        self.freq_dist = config["data_collection"]["frequencies"]["distance"]

        self.default_speed = config["movement"]["xy_speed"]
        self.default_distance = config["movement"]["distance"]
        self.default_z_speed = config["movement"]["z_speed"]
        self.default_angle = config["movement"]["angle"]

        os.makedirs(self.data_dir, exist_ok=True)
        date_str = datetime.now().strftime("%Y%m%d")

        pos_name = f"log_{date_str}_{config['data_collection']['files']['position']}.csv"
        att_name = f"log_{date_str}_{config['data_collection']['files']['attitude']}.csv"
        imu_name = f"log_{date_str}_{config['data_collection']['files']['imu']}.csv"
        esc_name = f"log_{date_str}_{config['data_collection']['files']['esc']}.csv"
        dist_name = f"log_{date_str}_{config['data_collection']['files']['distance']}.csv"

        self.pos_file = os.path.join(self.data_dir, pos_name)
        self.att_file = os.path.join(self.data_dir, att_name)
        self.imu_file = os.path.join(self.data_dir, imu_name)
        self.esc_file = os.path.join(self.data_dir, esc_name)
        self.dist_file = os.path.join(self.data_dir, dist_name)

    def save_to_csv(self, filename, data):
        current_time = time.time()
        with open(filename, mode="a", newline="") as f:
            writer = csv.writer(f)
            row = [current_time]
            for item in data:
                if isinstance(item, (list, tuple)):
                    row.extend(item)
                else:
                    row.append(item)
            writer.writerow(row)

    def handle_position(self, data):
        self.save_to_csv(self.pos_file, data)

    def handle_attitude(self, data):
        self.save_to_csv(self.att_file, data)
        self.current_yaw = data[0]

    def handle_imu(self, data):
        self.save_to_csv(self.imu_file, data)

    def handle_esc(self, data):
        self.save_to_csv(self.esc_file, data)

    def handle_distance(self, data):
        self.save_to_csv(self.dist_file, data)
        self.current_tof_dist_mm = data[0]

    def setup_csv_headers(self):
        with open(self.pos_file, mode="w", newline="") as f:
            csv.writer(f).writerow(["unix_timestamp", "x", "y", "z"])
        with open(self.att_file, mode="w", newline="") as f:
            csv.writer(f).writerow(["unix_timestamp", "yaw", "pitch", "roll"])
        with open(self.imu_file, mode="w", newline="") as f:
            csv.writer(f).writerow(["unix_timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])
        with open(self.esc_file, mode="w", newline="") as f:
            csv.writer(f).writerow(["unix_timestamp", "esc_data"])
        with open(self.dist_file, mode="w", newline="") as f:
            csv.writer(f).writerow(["unix_timestamp", "tof1"])

    def start_sensors(self):
        print("Starting to collect sensor data...")
        self.ep_chassis.sub_position(freq=self.freq_pos, callback=self.handle_position)
        self.ep_chassis.sub_attitude(freq=self.freq_att, callback=self.handle_attitude)
        self.ep_chassis.sub_imu(freq=self.freq_imu, callback=self.handle_imu)
        self.ep_chassis.sub_esc(freq=self.freq_esc, callback=self.handle_esc)
        self.ep_sensor.sub_distance(freq=self.freq_dist, callback=self.handle_distance)
        
        try:
            self.ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
            time.sleep(0.5)
        except Exception:
            pass

    def stop_sensors(self):
        time.sleep(self.buffer_time)
        self.ep_chassis.unsub_position()
        self.ep_chassis.unsub_attitude()
        self.ep_chassis.unsub_imu()
        self.ep_chassis.unsub_esc()
        self.ep_sensor.unsub_distance()
        cv2.destroyAllWindows()
        print("Data collection and saving to the file have been fully completed.")

    def reset_gimbal(self):
        """รีเซ็ต Gimbal กลับมาตรงกลางหน้าตรง (Yaw=0, Pitch=0) เพื่อความแม่นยำ"""
        try:
            self.ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=200).wait_for_completed()
            time.sleep(0.1)
        except Exception:
            pass

    def move_forward(self, distance=None, speed=None):
        if distance is None:
            distance = self.default_distance
        if speed is None:
            speed = self.default_speed
        self.ep_chassis.move(x=distance, y=0, z=0, xy_speed=speed).wait_for_completed()

    def safe_move_forward(self, distance=0.6, speed=0.3, stop_limit_mm=130):
        """เดินหน้าแบบปลอดภัยโดยใช้ drive_speed ควบคุมด้วยเวลา พร้อมรีเซ็ต Gimbal ก่อนเดิน"""
        print(f"--> [Safe Move] กำลังเดินหน้า {distance}m ด้วย drive_speed...")
        self.reset_gimbal()

        travel_time = distance / speed
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < travel_time:
                front_dist = self.current_tof_dist_mm
                
                if 0 < front_dist <= stop_limit_mm:
                    print(f"!!! [เบรกฉุกเฉิน] เจอสิ่งกีดขวางระยะ {front_dist}mm หยุดการทำงานทันที !!!")
                    self.ep_chassis.drive_speed(x=0, y=0, z=0)
                    time.sleep(0.5)
                    return False
                
                self.ep_chassis.drive_speed(x=speed, y=0, z=0)
                time.sleep(0.05)
                
        except Exception as e:
            print(f"[-] เกิดข้อผิดพลาดในการเคลื่อนที่: {e}")
            
        self.ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.3)
        return True

    def scan_surroundings_with_gimbal(self):
        """ใช้ Gimbal หมุนสแกนระยะ ToF หน้า ขวา ซ้าย (รีเซ็ตก่อนและหลังสแกนเสมอ)"""
        self.reset_gimbal()
        distances = {"front": 0, "right": 0, "left": 0}
        
        # 1. ด้านหน้า
        self.ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=150).wait_for_completed()
        time.sleep(0.15)
        distances["front"] = self.current_tof_dist_mm

        # 2. ด้านขวา
        self.ep_gimbal.moveto(pitch=0, yaw=90, yaw_speed=150).wait_for_completed()
        time.sleep(0.15)
        distances["right"] = self.current_tof_dist_mm

        # 3. ด้านซ้าย
        self.ep_gimbal.moveto(pitch=0, yaw=-90, yaw_speed=150).wait_for_completed()
        time.sleep(0.15)
        distances["left"] = self.current_tof_dist_mm

        # 4. กลับมาหน้าตรง
        self.reset_gimbal()
        return distances

    def draw_live_grid(self, current_pos, visited_set, max_x=3, max_y=3):
        """แสดงตำแหน่งตาราง Grid และตำแหน่งหุ่นแบบ Real-time"""
        cell_px = 120  
        width = (max_x + 1) * cell_px
        height = (max_y + 1) * cell_px
        
        img = np.ones((height, width, 3), dtype=np.uint8) * 255

        for r in range(max_y + 1):
            for c in range(max_x + 1):
                plot_y = max_y - r
                x1 = c * cell_px
                y1 = plot_y * cell_px
                x2 = x1 + cell_px
                y2 = y1 + cell_px

                if (c, r) in visited_set:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (229, 239, 247), -1)

                cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1)
                cv2.putText(img, f"({c},{r})", (x1 + 10, y1 + 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)

        cur_c, cur_r = current_pos
        plot_cur_y = max_y - cur_r
        center_x = cur_c * cell_px + cell_px // 2
        center_y = plot_cur_y * cell_px + cell_px // 2
        cv2.circle(img, (center_x, center_y), 30, (0, 0, 255), -1)
        cv2.putText(img, "ROBOT", (center_x - 26, center_y + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)

        cv2.imshow("SLAM Real-time Grid Monitor", img)
        cv2.waitKey(1)

    def explore_and_map_all(self):
        """อัลกอริทึมสำรวจพื้นที่แบบเต็มรูปแบบ (ไม่พึ่งพา Goal จอดที่ช่องสุดท้ายทันที)"""
        print("--- เริ่มการสำรวจและสร้างแผนที่ (Robust Grid Exploration) ---")

        data_cfg = self.config.get("data_collection", {})
        files_cfg = data_cfg.get("files", {})
        data_dir = data_cfg.get("data_dir", "data/raw/run1")
        os.makedirs(data_dir, exist_ok=True)
        date_str = datetime.now().strftime("%Y%m%d")

        filename_key = files_cfg.get("exploration", "exploration_map_data")
        csv_path = os.path.join(data_dir, f"log_{date_str}_{filename_key}.csv")

        with open(csv_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "unix_timestamp", "grid_x", "grid_y", 
                "real_x_m", "real_y_m", "heading", "heading_deg", 
                "front_tof_mm", "right_ir_cm", "left_ir_cm", "action"
            ])

        def log_step(g_x, g_y, h, tof_val, r_val, l_val, act_label="VISIT", c_size=0.6):
            deg_map = {0: 0, 1: 90, 2: 180, 3: 270}
            with open(csv_path, mode="a", newline="", encoding="utf-8") as file:
                c_writer = csv.writer(file)
                c_writer.writerow([
                    time.time(), g_x, g_y,
                    round(g_x * c_size, 3), round(g_y * c_size, 3),
                    h, deg_map.get(h, 0),
                    round(tof_val, 1), round(r_val / 10.0, 2), round(l_val / 10.0, 2), act_label
                ])

        if not hasattr(self, "current_tof_dist_mm"):
            self.current_tof_dist_mm = 10000

        CELL_SIZE = self.config.get("movement", {}).get("distance", 0.6)
        FRONT_WALL_MM = 300 
        SIDE_OPEN_MM = 450   

        visited = set()
        stack = []

        start_x, start_y = 0, 0
        x, y = start_x, start_y
        heading = 0  
        moves = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}

        grid_cfg = self.config.get("grid_map", {})
        MAX_X = grid_cfg.get("max_x", 3)
        MAX_Y = grid_cfg.get("max_y", 3)

        try:
            while True:
                self.ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.4)

                visited.add((x, y))
                print(f"\n[Map] พิกัดปัจจุบัน: ({x}, {y}) | ทิศหันหน้า: {heading}")

                self.draw_live_grid((x, y), visited, MAX_X, MAX_Y)

                # สแกนพื้นที่รอบตัว
                surrounding = self.scan_surroundings_with_gimbal()
                front_dist = surrounding["front"]
                right_dist = surrounding["right"]
                left_dist = surrounding["left"]

                log_step(x, y, heading, front_dist, right_dist, left_dist, act_label="VISIT", c_size=CELL_SIZE)

                # ตรวจสอบทิศทางที่เปิดอยู่
                open_dirs = []
                if front_dist > FRONT_WALL_MM:
                    open_dirs.append(heading)
                if right_dist > SIDE_OPEN_MM:
                    open_dirs.append((heading + 1) % 4)
                if left_dist > SIDE_OPEN_MM:
                    open_dirs.append((heading + 3) % 4)

                unvisited = []
                for d in open_dirs:
                    target_x = x + moves[d][0]
                    target_y = y + moves[d][1]
                    if 0 <= target_x <= MAX_X and 0 <= target_y <= MAX_Y:
                        if (target_x, target_y) not in visited:
                            unvisited.append(d)

                # ถ้ามีช่องใหม่ที่ยังไม่เคยไป ให้เดินหน้าไปช่องนั้น
                if unvisited:
                    next_heading = unvisited[0]
                    stack.append((x, y, heading))

                    turn_angle = (next_heading - heading) * 90
                    if turn_angle > 180: turn_angle -= 360
                    if turn_angle < -180: turn_angle += 360

                    if turn_angle == 90: self.turn_right(90)
                    elif turn_angle == -90: self.turn_left(90)
                    elif abs(turn_angle) == 180: self.turn_right(180)

                    time.sleep(0.3)

                    success = self.safe_move_forward(distance=CELL_SIZE)
                    if success:
                        x += moves[next_heading][0]
                        y += moves[next_heading][1]
                        heading = next_heading
                    else:
                        print("-> [Obstacle] ชนสิ่งกีดขวาง ยกเลิกเส้นทางนี้")
                        stack.pop()
                else:
                    # ถ้าไม่มีช่องใหม่ และ Stack หมด (สำรวจครบหมดแล้ว) -> หยุดทันทีที่ช่องสุดท้าย!
                    if not stack:
                        end_x, end_y = x, y
                        total_cells = (MAX_X + 1) * (MAX_Y + 1)
                        coverage_pct = round(100.0 * len(visited) / total_cells, 2)

                        print("\n" + "="*50)
                        print("📊 สรุปผลภารกิจ SLAM Explore สำเร็จ:")
                        print(f"   - จุดเริ่มต้น (Start Position): [{start_x}, {start_y}]")
                        print(f"   - จุดสิ้นสุด (End Position):   [{end_x}, {end_y}]")
                        print(f"   - พื้นที่สำรวจทั้งหมด (Coverage): {coverage_pct}% ({len(visited)}/{total_cells} cells)")
                        print("="*50 + "\n")
                        break

                    # ถอยกลับไปทางเดิมตาม Stack เพื่อหาช่องอื่นที่อาจยังค้างอยู่
                    prev_x, prev_y, prev_heading = stack.pop()
                    dx = prev_x - x
                    dy = prev_y - y
                    target_heading = 0
                    for h, m in moves.items():
                        if m == (dx, dy):
                            target_heading = h
                            break

                    turn_angle = (target_heading - heading) * 90
                    if turn_angle > 180: turn_angle -= 360
                    if turn_angle < -180: turn_angle += 360

                    if turn_angle == 90: self.turn_right(90)
                    elif turn_angle == -90: self.turn_left(90)
                    elif abs(turn_angle) == 180: self.turn_right(180)

                    self.safe_move_forward(distance=CELL_SIZE)
                    x, y = prev_x, prev_y
                    heading = target_heading

                    log_step(x, y, heading, self.current_tof_dist_mm, 300.0, 300.0, act_label="RETRACE", c_size=CELL_SIZE)

        except KeyboardInterrupt:
            print("\n--> ยกเลิกการสำรวจโดยผู้ใช้")
            self.ep_chassis.drive_speed(x=0, y=0, z=0)

        total_cells = (MAX_X + 1) * (MAX_Y + 1)
        coverage_pct = round(100.0 * len(visited) / total_cells, 2) if total_cells else 0.0

        report = {
            "start_grid": [start_x, start_y],
            "end_grid": [x, y],
            "visited_cells": len(visited),
            "total_grid_cells": total_cells,
            "coverage_percent": coverage_pct,
            "exploration_log_csv": csv_path,
        }
        return report

    def turn_left(self, angle=None, speed=None):
        if angle is None:
            angle = self.default_angle
        if speed is None:
            speed = self.default_z_speed
        self.ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.3)
        self.ep_chassis.move(x=0, y=0, z=angle, z_speed=speed).wait_for_completed()
        time.sleep(0.3)

    def turn_right(self, angle=None, speed=None):
        if angle is None:
            angle = self.default_angle
        if speed is None:
            speed = self.default_z_speed
        self.ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.3)
        self.ep_chassis.move(x=0, y=0, z=-angle, z_speed=speed).wait_for_completed()
        time.sleep(0.3)