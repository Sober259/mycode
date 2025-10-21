"""
 * File: multi_offb_node.py
 * Fixed version for multiple UAV offboard control
"""

#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time
import threading

class UAVController:
    def __init__(self, uav_namespace):
        self.namespace = uav_namespace
        self.current_state = State()
        self.connected = False
        self.armed = False
        self.offboard_enabled = False
        
        # 使用完整的命名空间路径
        state_topic = f"/{uav_namespace}/mavros/state"
        setpoint_topic = f"/{uav_namespace}/mavros/setpoint_position/local"
        arming_service = f"/{uav_namespace}/mavros/cmd/arming"
        set_mode_service = f"/{uav_namespace}/mavros/set_mode"
        
        self.state_sub = rospy.Subscriber(state_topic, State, self.state_cb)
        self.local_pos_pub = rospy.Publisher(setpoint_topic, PoseStamped, queue_size=10)

        # 等待服务可用
        rospy.wait_for_service(arming_service)
        rospy.wait_for_service(set_mode_service)
        
        self.arming_client = rospy.ServiceProxy(arming_service, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(set_mode_service, SetMode)

        self.rate = rospy.Rate(20)  # 20Hz

    def state_cb(self, msg):
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed

    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # 保持水平姿态
        pose.pose.orientation.w = 1.0
        return pose

    def initialize(self):
        """等待连接并发送初始SetPoint"""
        rospy.loginfo(f"{self.namespace}: Waiting for connection...")
        while not rospy.is_shutdown() and not self.connected:
            self.rate.sleep()
        
        rospy.loginfo(f"{self.namespace}: Connected! Sending initial setpoints...")
        
        # 发送初始位置（当前位置）
        initial_pose = self.create_pose(0, 0, 0)
        
        # 持续发送SetPoint至少5秒
        start_time = time.time()
        while time.time() - start_time < 5.0 and not rospy.is_shutdown():
            initial_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(initial_pose)
            self.rate.sleep()

    def set_offboard_mode(self):
        """尝试设置OFFBOARD模式"""
        try:
            resp = self.set_mode_client(0, 'OFFBOARD')  # 0表示base_mode, 'OFFBOARD'表示custom_mode
            if resp.mode_sent:
                rospy.loginfo(f"{self.namespace}: OFFBOARD mode set successfully")
                self.offboard_enabled = True
                return True
            else:
                rospy.logwarn(f"{self.namespace}: Failed to set OFFBOARD mode")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"{self.namespace}: Set mode service call failed: {e}")
            return False

    def arm_vehicle(self):
        """尝试解锁无人机"""
        try:
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo(f"{self.namespace}: Vehicle armed successfully")
                return True
            else:
                rospy.logwarn(f"{self.namespace}: Failed to arm vehicle")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"{self.namespace}: Arming service call failed: {e}")
            return False

    def control_loop(self, target_x, target_y, target_z):
        """主控制循环"""
        target_pose = self.create_pose(target_x, target_y, target_z)
        
        # 初始化
        self.initialize()
        
        last_request = rospy.Time.now()
        setpoint_start_time = rospy.Time.now()
        
        rospy.loginfo(f"{self.namespace}: Starting control loop...")

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 持续发送SetPoint（这是关键！）
            target_pose.header.stamp = current_time
            self.local_pos_pub.publish(target_pose)
            
            # 确保SetPoint已经发送了一段时间（至少2秒）
            if (current_time - setpoint_start_time).to_sec() > 2.0:
                # 尝试设置OFFBOARD模式（每5秒尝试一次）
                if not self.offboard_enabled and (current_time - last_request).to_sec() > 5.0:
                    if self.set_offboard_mode():
                        last_request = current_time
                    else:
                        last_request = current_time
                
                # 如果OFFBOARD模式已设置但未解锁，尝试解锁
                elif self.offboard_enabled and not self.armed and (current_time - last_request).to_sec() > 5.0:
                    if self.arm_vehicle():
                        last_request = current_time
                    else:
                        last_request = current_time
            
            self.rate.sleep()

class MultiUAVControl:
    def __init__(self, uav_namespaces):
        rospy.init_node("multi_offb_node_py", anonymous=False)
        self.uav_controllers = []
        
        for namespace in uav_namespaces:
            controller = UAVController(namespace)
            self.uav_controllers.append(controller)
    
    def run_formation(self, formation_positions):
        """运行编队控制"""
        threads = []
        
        # 为每个无人机创建控制线程
        for i, controller in enumerate(self.uav_controllers):
            if i < len(formation_positions):
                x, y, z = formation_positions[i]
                thread = threading.Thread(
                    target=controller.control_loop, 
                    args=(x, y, z),
                    name=f"{controller.namespace}_thread"
                )
                thread.daemon = True
                threads.append(thread)
        
        # 启动所有线程
        for thread in threads:
            thread.start()
            rospy.loginfo(f"Started thread: {thread.name}")
        
        # 等待ROS关闭或线程结束
        try:
            while not rospy.is_shutdown():
                time.sleep(0.1)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        
        # 等待所有线程结束
        for thread in threads:
            thread.join(timeout=1.0)

if __name__ == "__main__":
    try:
        import sys
        import math

        # 通过命令行参数传入无人机数量 (例如: rosrun test multi_takeoff_node.py 6)
        num_uavs = int(sys.argv[1]) if len(sys.argv) > 1 else 3

        # 自动生成命名空间
        uav_namespaces = [f"uav{i}" for i in range(num_uavs)]

        multi_controller = MultiUAVControl(uav_namespaces)

        # === 生成编队位置 ===
        def generate_circle_positions(num_uavs, altitude=5):
            """根据无人机数量生成圆形分布位置"""
            positions = []
            radius = int (num_uavs)
            if num_uavs == 0:
                return []
            # uav0 在 (0, 0, z)
            positions.append((0, 0, altitude))

            if num_uavs > 1:
                angle_step = 2 * math.pi / num_uavs
                n = 0
                i = 1
                while i < num_uavs + 1:
                    n += 1
                    angle = n * angle_step
                    x = radius * math.sin(angle)
                    y = radius - radius * math.cos(angle) - 3 * i
                    positions.append((x, y, altitude))
                    i += 1
                    if i < num_uavs + 1:
                        x = radius * math.sin(angle) * -1
                        y = radius - radius * math.cos(angle) - 3 * i
                        positions.append((x, y, altitude))
                        i += 1

                

            return positions
    

        formation_positions = generate_circle_positions(num_uavs, altitude=5)

        rospy.loginfo(f"Generated {num_uavs} UAV target positions:")
        for i, p in enumerate(formation_positions):
            rospy.loginfo(f"UAV{i}: {p}")

        # multi_controller = MultiUAVControl(uav_namespaces)
        multi_controller.run_formation(formation_positions)

    except rospy.ROSInterruptException:
        pass
