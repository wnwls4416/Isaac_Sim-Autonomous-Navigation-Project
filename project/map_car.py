#!/usr/bin/env python3
import sys
import numpy as np

# 1. Isaac Sim 엔진 실행
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# =========================================================
# ✨ 핵심: ROS 2 브릿지 익스텐션 활성화 및 로드 대기
from isaacsim.core.utils.extensions import enable_extension

# 익스텐션을 켜고 시뮬레이터가 완전히 로드될 때까지 한 번 업데이트합니다.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update() 
# =========================================================

# 익스텐션이 안전하게 로드된 이후에 rclpy를 import 합니다.
import rclpy
from rclpy.node import Node

# Isaac Sim Core API 및 객체 제어 모듈 임포트
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim

def main(args=None):
    rclpy.init(args=args)
    node = Node('isaac_sim_map_node')
    
    world = World()
    
    # 1. 조명 세팅
    node.get_logger().info("하늘에 조명을 켭니다 💡")
    create_prim(
        prim_path="/World/SkyLight",
        prim_type="DomeLight",
        attributes={
            "inputs:intensity": 1000.0,
            "inputs:color": (1.0, 1.0, 1.0)
        }
    )
    
    # 2. 맵 소환 및 위치 고정
    map_usd_path = "/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/project/resource/map.usd"
    map_prim_path = "/World/Map"
    
    node.get_logger().info(f"맵 소환 중: {map_usd_path}")
    add_reference_to_stage(usd_path=map_usd_path, prim_path=map_prim_path)
    
    # 맵의 좌표를 [0, 0, 0]으로 명시적 강제 고정
    map_prim = SingleXFormPrim(
        prim_path=map_prim_path,
        name="map_prim",
        position=np.array([0.0, 0.0, 0.0])
    )

    # 3. 차량 소환
    car_usd_path = "/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/project/resource/ackermann_car_fixed_cam.usd"
    car_prim_path = "/World/ackermann_car"
    
    node.get_logger().info(f"자동차 소환 중: {car_usd_path}")
    add_reference_to_stage(usd_path=car_usd_path, prim_path=car_prim_path)
    
    # 물리 폭발(자유낙하)을 방지하기 위해 Z축 높이를 0.5로 설정하여 스폰
    car = SingleArticulation(
        prim_path=car_prim_path, 
        name="ackermann_car_prim",
        position=np.array([14.3206, -20.01481, 4.13206]),
        orientation=np.array([0.7071068, 0.0, 0.0, 0.7071068]) 
    )
    
    world.scene.add(car)
    world.reset()
    
    node.get_logger().info("차량 및 맵 안전하게 스폰 완료! ROS 2 통신 대기 중...")

    # 4. 실행 루프
    while simulation_app.is_running() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        world.step(render=True)

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == '__main__':
    main()