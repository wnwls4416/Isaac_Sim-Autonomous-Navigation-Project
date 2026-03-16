#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped 
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String  
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import heapq

# 추가된 라이브러리
import matplotlib.pyplot as plt
import networkx as nx
import tkinter as tk
from tkinter import simpledialog
import threading  # 🌟 스레드 분리를 위한 추가

# --- 1. 맵 데이터베이스 및 A* 알고리즘 ---
NODES = {
    'start': (14.3674, -20.01481), 
    'intersection_1': (13.2995, 5.47381),
    'intersection_2': (14.3674, 5.38333),
    'intersection_3': (12.67477, 4.50676),
    'intersection_4': (15.15383, 4.44766), 
    'intersection_5': (15.38423, 3.47689),
    'intersection_6': (15.23355, 3.46273), 
    'intersection_7': (13.27094, 2.56761),
    'intersection_8': (14.3674, 2.59151),
    'intersection_9': (15.28878, -1.78882),
    'intersection_10': (23.19041, -0.68786), 
    'intersection_11': (22.33719, -1.7952),
    'intersection_12': (23.99006, -1.7952), 
    'intersection_13': (23.11583, -2.58109),
    'intersection_14': (15.52753, 12.91614), 
    'intersection_15': (-19.1235, 4.46522),
    'intersection_16': (-20.02404, -21.81379),
    'intersection_17': (-20.00033, 3.68375),
    'intersection_18': (23.14985, 5.15102), 
    'intersection_19': (23.25342, 2.86142),
    'intersection_20': (14.28422, -2.49098),
    'intersection_21': (14.30677, 11.96846),
    'intersection_22': (4.20905, 4.54039),
    'intersection_23': (3.31402, 5.86453),
    'intersection_24': (3.31402, 14.8177),
    'intersection_25': (3.960091, 15.62502),
    'intersection_26': (12.3968, 15.62502),
    'intersection_27': (13.37456, 15.0475),
    'intersection_28': (22.26219, 4.5053),
    'intersection_29': (22.31807, 3.48337),
    'intersection_30': (23.82057, 3.48337),
    'intersection_31': (22.61432, 12.87311),
    'intersection_32': (23.12922, 12.26361),
    'intersection_33': (22.33329, 4.5065),
    'intersection_34': (-19.57721, -22.47848),
    'fire_station': (14.35285, -6.15644),
    'helipad': (9.28755, 4.49427),
    'factory': (7.21136, -22.45507),
    'starbucks': (25.66021, -1.77709),
    'gas_station': (19.58518, 4.51724),
    'city_hall': (26.2449, 3.45905),
    'river' : (-20.06203, -11.92419),
    'hospital': (21.44779, 12.87311),
    'opistel': (19.93647, -1.78882),
    'orkqn_coffee': (23.13512, 0.68248),
    'crosswalk': (3.3662, 10.12544),
    'home': (14.3701, 9.68075),
    'village': (19.94294, 3.4744),
    'town': (6.91746, 15.60958),
    'center_1': (13.86396, 4.00312),
    'hotel' : (13.37456, 13.06892),
    'parking': (23.12922, 8.98442)
}

GRAPH = {
    'start': ['intersection_8', 'fire_station', 'intersection_20'],
    'intersection_1': ['center_1'],
    'intersection_2': ['center_1', 'home'],
    'intersection_3': ['center_1', 'helipad'],
    'intersection_4': ['gas_station'],
    'intersection_5': ['center_1', 'intersection_17'],
    'intersection_6': ['intersection_8', 'intersection_19', 'city_hall'],
    'intersection_7': ['center_1'],
    'intersection_8': ['center_1', 'intersection_6', 'intersection_2', 'fire_station'],
    'intersection_9': ['intersection_8', 'intersection_20', 'opistel'],
    'intersection_10': ['intersection_11', 'intersection_12','intersection_13', 'orkqn_coffee'],
    'intersection_11': ['intersection_10','intersection_12','intersection_13', 'opistel'],
    'intersection_12': ['intersection_10','intersection_11','intersection_13', 'starbucks'],
    'intersection_13': ['intersection_10','intersection_11','intersection_12'],
    'intersection_14': ['intersection_2', 'hospital'],
    'intersection_15': ['intersection_17', 'helipad'],
    'intersection_16': ['intersection_17', 'intersection_34'],
    'intersection_17': ['intersection_15', 'river'],
    'intersection_18': ['intersection_33', 'parking'],
    'intersection_19': ['intersection_6', 'intersection_30', 'orkqn_coffee'],
    'intersection_20': ['fire_station', 'intersection_9'],
    'intersection_21': ['intersection_14', 'home'],
    'intersection_22': ['intersection_23'],
    'intersection_23': ['intersection_22', 'crosswalk'],
    'intersection_24': ['intersection_25', 'crosswalk'],
    'intersection_25': ['intersection_24', 'town'],
    'intersection_26': ['intersection_27', 'town'],
    'intersection_27': ['intersection_26', 'hotel'],
    'intersection_28': ['intersection_18', 'gas_station'],
    'intersection_29': ['intersection_19', 'village'],
    'intersection_30': ['intersection_19', 'city_hall'],
    'intersection_31': ['intersection_32', 'hospital'],
    'intersection_32': ['intersection_31', 'parking'],
    'intersection_33': ['intersection_18', 'gas_station'],
    'intersection_34': ['intersection_16', 'factory'],
    'fire_station': ['start', 'intersection_8','intersection_20'],
    'helipad': ['intersection_3', 'intersection_15', 'intersection_22'],
    'factory': ['intersection_34'],
    'starbucks': ['intersection_12'],
    'gas_station': ['intersection_4', 'intersection_33'],
    'city_hall': ['intersection_9'],
    'hospital': ['intersection_14', 'intersection_31'],
    'opistel': ['intersection_9', 'intersection_11'],
    'orkqn_coffee': ['intersection_19', 'intersection_10'],
    'crosswalk': ['intersection_23', 'intersection_24'],
    'home': ['intersection_2', 'intersection_21'],
    'river': ['intersection_16', 'intersection_17'],
    'village': ['intersection_6', 'intersection_19'],
    'center_1': ['intersection_1', 'intersection_2', 'intersection_3', 'intersection_5', 'intersection_6', 'intersection_7'],
    'town': ['intersection_25', 'intersection_26'],
    'hotel': ['intersection_27'],
    'parking': ['intersection_18', 'intersection_32']
}

def calculate_distance(node1, node2):
    x1, y1 = NODES[node1]
    x2, y2 = NODES[node2]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def a_star_search(start_node, goal_node):
    queue = []
    heapq.heappush(queue, (0, start_node, [start_node]))
    g_costs = {start_node: 0}
    
    while queue:
        _, current_node, path = heapq.heappop(queue)
        if current_node == goal_node:
            return path
        for neighbor in GRAPH[current_node]:
            tentative_g = g_costs[current_node] + calculate_distance(current_node, neighbor)
            if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                g_costs[neighbor] = tentative_g
                f_cost = tentative_g + calculate_distance(neighbor, goal_node)
                heapq.heappush(queue, (f_cost, neighbor, path + [neighbor]))
    return None

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class AutonomousNavNode(Node):
    def __init__(self):
        super().__init__('autonomous_nav_node')
        
        self.spawn_x = 14.3674
        self.spawn_y = -20.01481
        self.spawn_yaw = math.pi / 2  

        self.current_node = 'start' 
        self.goal_node = None
        self.path = []
        self.is_goal_reached = True 
        self.target_idx = 0
        
        self.current_x = self.spawn_x
        self.current_y = self.spawn_y
        self.current_yaw = self.spawn_yaw
        
        self.lane_offset = 0.0     
        self.line_detected = False 
        self.last_known_steer = 0.0 
        
        self.fixed_turn_steer = 0.0
        self.nav_mode = 'VISION'

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.overlay_pub = self.create_publisher(Image, '/camera_left/lane_overlay', 10)
        self.image_sub = self.create_subscription(Image, '/camera_left/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.goal_sub = self.create_subscription(String, '/set_goal', self.goal_callback, 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.control_loop)
        self.debug_timer = self.create_timer(2.0, self.debug_print_location)
        
        # 🌟 맵 초기화 실행 (하지만 ROS 타이머는 쓰지 않음)
        self.init_map()

        self.get_logger().info("🚗 자율주행 노드가 시작되었습니다. 목적지 토픽 대기 중...")

    def init_map(self):
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.G = nx.DiGraph()

        for node, pos in NODES.items():
            self.G.add_node(node, pos=pos)
        for source, targets in GRAPH.items():
            for target in targets:
                self.G.add_edge(source, target)

        self.pos = nx.get_node_attributes(self.G, 'pos')
        
        node_colors = []
        for node in self.G.nodes():
            if 'intersection' in node: node_colors.append('lightgray')
            elif node == 'start': node_colors.append('green')
            else: node_colors.append('skyblue')

        nx.draw(self.G, self.pos, ax=self.ax, with_labels=False, node_size=300, 
                node_color=node_colors, edge_color='gray', arrows=True, arrowsize=10, alpha=0.6)
        
        labels_pos = {k: (v[0], v[1]+0.8) for k, v in self.pos.items()}
        nx.draw_networkx_labels(self.G, labels_pos, font_size=8, font_weight='bold', ax=self.ax)

        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Path', alpha=0.5)
        self.car_marker, = self.ax.plot([self.current_x], [self.current_y], 'ro', markersize=10, label='My Car')

        self.ax.legend()
        plt.title("Real-Time Autonomous Navigation Map", fontsize=14)
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.tight_layout()

    def update_map(self):
        # 🌟 UI 스레드에서만 호출되며, 캔버스 그리기(draw/flush)를 제거하여 최적화
        self.car_marker.set_data([self.current_x], [self.current_y])
        
        if self.path and not self.is_goal_reached and self.target_idx < len(self.path):
            path_x = [self.current_x] + [NODES[n][0] for n in self.path[self.target_idx:]]
            path_y = [self.current_y] + [NODES[n][1] for n in self.path[self.target_idx:]]
            self.path_line.set_data(path_x, path_y)
        else:
            self.path_line.set_data([], [])

    def goal_callback(self, msg):
        new_goal = msg.data
        if "intersection" in new_goal or new_goal in ['center_1', 'crosswalk']:
            self.get_logger().warning(f"⚠️ [{new_goal}]은(는) 도착지로 설정할 수 없는 곳입니다.")
            return

        if new_goal not in NODES:
            self.get_logger().error(f"❌ [{new_goal}] 노드가 지도(NODES)에 존재하지 않습니다.")
            return

        if not self.is_goal_reached:
            self.get_logger().warning("🚗 현재 이동 중입니다. 목적지 도착 후 다시 시도해주세요.")
            return

        self.get_logger().info(f"🎯 새로운 목적지 수신: [{new_goal}]")
        self.goal_node = new_goal
        self.path = a_star_search(self.current_node, self.goal_node)
        
        if self.path is None:
            self.get_logger().error("❌ 경로를 찾을 수 없습니다!")
            return
            
        self.get_logger().info(f"🗺️ 경로 생성 완료: {' -> '.join(self.path)}")
        self.target_idx = 1
        self.is_goal_reached = False 
        self.update_navigation_state(self.path[0], self.path[1])

    def debug_print_location(self):
        if self.is_goal_reached or len(self.path) == 0: return
        target_node = self.path[self.target_idx]
        tx, ty = NODES[target_node]
        dist = math.sqrt((tx - self.current_x)**2 + (ty - self.current_y)**2)
        
        self.get_logger().info(f"📡 [위치 추적] 현재 좌표: ({self.current_x:.2f}, {self.current_y:.2f}) | 다음 목표: {target_node} ({dist:.2f}m)")

    def update_navigation_state(self, prev_node, new_target):
        self.get_logger().info(f"📍 현재 [ {prev_node} ] 통과 중 -> 다음 목표: [ {new_target} ]")

        dist = calculate_distance(prev_node, new_target)
        is_center_involved = "center" in prev_node or "center" in new_target
        is_short_crossing = ("intersection" in prev_node and "intersection" in new_target and dist < 3.5)
        
        if is_center_involved or is_short_crossing:
            self.nav_mode = 'BLIND'
            tx, ty = NODES[new_target]
            px, py = NODES[prev_node]
            
            target_angle = math.atan2(ty - self.current_y, tx - self.current_x)
            h_err = target_angle - self.current_yaw
            
            while h_err > math.pi: h_err -= 2 * math.pi
            while h_err < -math.pi: h_err += 2 * math.pi
            
            delta_x = abs(tx - px)
            delta_y = abs(ty - py)
            
            is_axis_straight = (delta_x < 1.0 or delta_y < 1.0) and abs(h_err) < 0.1745
            
            if is_axis_straight:
                self.fixed_turn_steer = 0.0
                self.get_logger().info("⬆️ 방향 감지: X/Y축 궤적 직진 확정! -> 교차로 횡단 직진 (조향 0.0)")
            elif h_err > 0.05:  
                if is_center_involved:
                    self.fixed_turn_steer = 0.3
                    self.get_logger().info("↩️ 방향 감지: 좌측 -> Type 3 (Center 관통 좌회전, 조향 0.15)")
                else:
                    self.fixed_turn_steer = 0.34  
                    self.get_logger().info("↩️ 방향 감지: 좌측 -> Type 2 (일반 좌회전, 조향 0.34)")
            elif h_err < -0.05: 
                if is_center_involved:
                    self.fixed_turn_steer = -0.3
                    self.get_logger().info("↪️ 방향 감지: 우측 -> Type 3 (Center 관통 우회전, 조향 -0.08)")
                else:
                    self.fixed_turn_steer = -0.35
                    self.get_logger().info("↪️ 방향 감지: 우측 -> Type 1 (직각 우회전, 조향 -0.45)")
            else:
                self.fixed_turn_steer = 0.0
                self.get_logger().info(f"⬆️ 방향 감지: 정면 -> 교차로 횡단 직진 (조향 0.0, 거리: {dist:.1f}m)")
        else:
            self.nav_mode = 'VISION'
            self.get_logger().info(f"👀 장거리 직진 도로 진입 (VISION) 시작 (거리: {dist:.1f}m)")

    def odom_callback(self, msg):
        if self.is_goal_reached or len(self.path) == 0: return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0: return

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        odom_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        if not hasattr(self, 'odom_is_absolute'):
            if abs(odom_x) > 5.0 or abs(odom_y) > 5.0:
                self.odom_is_absolute = True
                self.get_logger().info("🤖 [시스템] /odom이 '절대 좌표(World)'로 들어오고 있습니다.")
            else:
                self.odom_is_absolute = False
                self.get_logger().info("🤖 [시스템] /odom이 '상대 좌표(Relative)'로 들어오고 있습니다.")

        if self.odom_is_absolute:
            self.current_x = odom_x
            self.current_y = odom_y
            self.current_yaw = odom_yaw
        else:
            self.current_x = self.spawn_x + (odom_x * math.cos(self.spawn_yaw) - odom_y * math.sin(self.spawn_yaw))
            self.current_y = self.spawn_y + (odom_x * math.sin(self.spawn_yaw) + odom_y * math.cos(self.spawn_yaw))
            self.current_yaw = self.spawn_yaw + odom_yaw

        while self.current_yaw > math.pi: self.current_yaw -= 2 * math.pi
        while self.current_yaw < -math.pi: self.current_yaw += 2 * math.pi

        target_node = self.path[self.target_idx]
        tx, ty = NODES[target_node]
        
        dist_to_target = math.sqrt((tx - self.current_x)**2 + (ty - self.current_y)**2)

        tolerance = 0.8 
        if "center" in target_node:
            tolerance = 1.0 

        if dist_to_target < tolerance:
            self.get_logger().info(f"✅ 목표 노드 [ {target_node} ] 진입 완료! (오차: {dist_to_target:.2f}m)")
            
            self.target_idx += 1
            
            if self.target_idx >= len(self.path):
                self.get_logger().info(f"🎉 최종 목적지 [{self.goal_node}]에 도착했습니다! 다음 목적지를 기다립니다.")
                
                self.current_node = self.goal_node 
                self.is_goal_reached = True 
                return
            
            new_target = self.path[self.target_idx]
            prev_node = self.path[self.target_idx - 1]
            
            self.update_navigation_state(prev_node, new_target)

    def image_callback(self, msg):
        if self.is_goal_reached: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception: return

        height, width = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        roi_mask = np.zeros_like(blue_mask)
        
        vertices = np.array([[
            (int(width * 0.1), height),               
            (int(width * 0.35), int(height * 0.6)),   
            (int(width * 0.65), int(height * 0.6)),   
            (int(width * 0.9), height)                
        ]], dtype=np.int32)
        
        cv2.fillPoly(roi_mask, vertices, 255)
        
        masked_blue = cv2.bitwise_and(blue_mask, roi_mask)
        contours, _ = cv2.findContours(masked_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        image_center = width // 2
        
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                self.lane_offset = cx - image_center
                self.line_detected = True
                
                cv2.circle(cv_image, (cx, cy), 15, (0, 0, 255), -1) 
                cv2.line(cv_image, (image_center, cy), (cx, cy), (0, 255, 255), 3) 
                
            else: self.line_detected = False
        else: self.line_detected = False

        cv2.polylines(cv_image, [vertices], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.line(cv_image, (image_center, 0), (image_center, height), (0, 255, 0), 2)

        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.overlay_pub.publish(overlay_msg)
        except: pass

    def control_loop(self):
        drive_msg = AckermannDriveStamped()
        
        if self.is_goal_reached:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.cmd_pub.publish(drive_msg)
            return

        max_steer_angle = 0.523599

        if self.nav_mode == 'BLIND':
            steer = self.fixed_turn_steer
            drive_msg.drive.speed = 0.6  
            
        elif self.nav_mode == 'VISION':
            if self.line_detected:
                error = self.lane_offset
                
                if error < -250:
                    steer = 0.40;  drive_msg.drive.speed = 0.2
                elif error < -160:
                    steer = 0.25;  drive_msg.drive.speed = 0.4
                elif error < -110:
                    steer = 0.15;  drive_msg.drive.speed = 0.6
                elif error < -75:
                    steer = 0.08;  drive_msg.drive.speed = 1.0
                elif error < -50:
                    steer = 0.04;  drive_msg.drive.speed = 1.4
                elif error < -30:
                    steer = 0.02;  drive_msg.drive.speed = 1.7   
                elif error < -15:
                    steer = 0.01;  drive_msg.drive.speed = 2.0   
                elif error <= 15:
                    steer = 0.0;   drive_msg.drive.speed = 2.2   
                elif error <= 30:
                    steer = -0.01; drive_msg.drive.speed = 2.0   
                elif error <= 50:
                    steer = -0.02; drive_msg.drive.speed = 1.7   
                elif error <= 75:
                    steer = -0.04; drive_msg.drive.speed = 1.4
                elif error <= 110:
                    steer = -0.08; drive_msg.drive.speed = 1.0
                elif error <= 160:
                    steer = -0.15; drive_msg.drive.speed = 0.6
                elif error <= 250:
                    steer = -0.25; drive_msg.drive.speed = 0.4
                else:
                    steer = -0.40; drive_msg.drive.speed = 0.2

                self.last_known_steer = steer
            else:
                steer = self.last_known_steer * 1.2 
                drive_msg.drive.speed = 0.3 

        steer = max(-max_steer_angle, min(steer, max_steer_angle))
            
        drive_msg.drive.steering_angle = float(steer)
        self.cmd_pub.publish(drive_msg)

def main(args=None):
    # 1. UI 창 띄우기 (입력 완료 전까지 ROS 실행 대기)
    root = tk.Tk()
    root.withdraw() 
    initial_goal = simpledialog.askstring("네비게이션 목적지 설정", "이동할 도착지를 입력하세요 (예: fire_station, home, opistel):")
    root.destroy()
    
    rclpy.init(args=args)
    node = AutonomousNavNode()
    
    if initial_goal:
        msg = String()
        msg.data = initial_goal
        node.goal_callback(msg)
        
    # 🌟 2. ROS 자율주행 루프를 서브 스레드로 분리 (병목 방지)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 🌟 3. 메인 스레드에서는 화면 UI(지도) 새로고침만 전담
    try:
        while rclpy.ok():
            node.update_map()   # 데이터 갱신
            plt.pause(0.1)      # 0.1초마다 GUI 이벤트 처리 (화면 새로고침)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()