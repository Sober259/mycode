#!/usr/bin/env python3

import rospy
import time
import sys
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import threading
import os

class UAVControl:
    def __init__(self, uav_id):
        self.uav_id = uav_id
        self.namespace = f"uav{uav_id}"
        
        # 初始化当前状态
        self.current_state = State()
        self.connected = False
        self.armed = False
        self.mode = ""
        self.local_pose = PoseStamped()
        self.offboard_enabled = False

        # 初始化订阅者
        self.state_sub = rospy.Subscriber(
            f"/{self.namespace}/mavros/state", 
            State, 
            self.state_cb
        )
        self.local_pos_sub = rospy.Subscriber(
            f"/{self.namespace}/mavros/local_position/pose", 
            PoseStamped, 
            self.pose_cb
        )

        # 初始化发布者
        self.local_pos_pub = rospy.Publisher(
            f"/{self.namespace}/mavros/setpoint_position/local", 
            PoseStamped, 
            queue_size=10
        )

        # 初始化服务客户端
        try:
            rospy.wait_for_service(f"/{self.namespace}/mavros/set_mode", timeout=10)
            self.set_mode_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/set_mode", SetMode)
        except rospy.ROSException:
            rospy.logwarn(f"UAV {self.uav_id}: Set mode service not available, will retry")
            self.set_mode_client = None

        try:
            rospy.wait_for_service(f"/{self.namespace}/mavros/cmd/arming", timeout=10)
            self.arming_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/cmd/arming", CommandBool)
        except rospy.ROSException:
            rospy.logwarn(f"UAV {self.uav_id}: Arming service not available, will retry")
            self.arming_client = None

        self.rate = rospy.Rate(20)
        rospy.loginfo(f"Initialized controller for UAV {uav_id}")

    def state_cb(self, msg):
        """状态回调函数"""
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

    def pose_cb(self, msg):
        """本地位姿回调"""
        self.local_pose = msg

    def create_pose(self, x, y, z):
        """创建位姿消息"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def wait_for_connection(self, timeout=30):
        """等待无人机连接"""
        start_time = time.time()
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if self.connected:
                rospy.loginfo(f"UAV {self.uav_id}: Connected to FCU")
                return True
            self.rate.sleep()
        rospy.logwarn(f"UAV {self.uav_id}: Connection timeout")
        return False

    def set_offboard_mode(self):
        """设置OFFBOARD模式"""
        if self.set_mode_client is None:
            try:
                rospy.wait_for_service(f"/{self.namespace}/mavros/set_mode", timeout=5)
                self.set_mode_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/set_mode", SetMode)
            except rospy.ROSException:
                rospy.logerr(f"UAV {self.uav_id}: Set mode service still not available")
                return False
        
        try:
            resp = self.set_mode_client(0, 'OFFBOARD')
            if resp.mode_sent:
                rospy.loginfo(f"UAV {self.uav_id}: OFFBOARD mode enabled")
                self.offboard_enabled = True
                return True
            else:
                rospy.logwarn(f"UAV {self.uav_id}: Failed to set OFFBOARD mode")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"UAV {self.uav_id}: Set mode service call failed: {e}")
            return False

    def arm_vehicle(self):
        """解锁无人机"""
        if self.arming_client is None:
            try:
                rospy.wait_for_service(f"/{self.namespace}/mavros/cmd/arming", timeout=5)
                self.arming_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/cmd/arming", CommandBool)
            except rospy.ROSException:
                rospy.logerr(f"UAV {self.uav_id}: Arming service still not available")
                return False
        
        try:
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo(f"UAV {self.uav_id}: Armed successfully")
                return True
            else:
                rospy.logwarn(f"UAV {self.uav_id}: Failed to arm vehicle")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"UAV {self.uav_id}: Arming service call failed: {e}")
            return False

    def send_setpoint(self, x, y, z):
        """发送位置设定点"""
        pose = self.create_pose(x, y, z)
        self.local_pos_pub.publish(pose)

    def is_ready_for_formation(self):
        """检查无人机是否准备好进行编队"""
        return self.connected and self.armed and self.mode == "OFFBOARD"

class MultiUAVFormationControl:
    def __init__(self, num_uavs):
        rospy.init_node("multi_uav_formation_control", anonymous=False)
        self.num_uavs = num_uavs
        self.uav_controllers = []
        
        # 初始化所有无人机控制器
        for i in range(num_uavs):
            controller = UAVControl(i)
            self.uav_controllers.append(controller)
        
        self.formation_active = False
        self.formation_thread = None
        
        rospy.loginfo(f"Initialized MultiUAVFormationControl with {num_uavs} UAVs")

    def wait_for_all_connections(self, timeout=30):
        """等待所有无人机连接"""
        rospy.loginfo("Waiting for all UAVs to connect...")
        start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            all_connected = all(uav.connected for uav in self.uav_controllers)
            if all_connected:
                rospy.loginfo("All UAVs connected!")
                return True
            
            # 打印连接状态
            connected_count = sum(1 for uav in self.uav_controllers if uav.connected)
            if time.time() - start_time > 5:  # 5秒后开始定期报告
                rospy.loginfo(f"Connected: {connected_count}/{self.num_uavs}")
            time.sleep(1)
        
        rospy.logwarn("Not all UAVs connected within timeout")
        return False

    def check_takeoff_node_exists(self):
        """检查起飞节点是否存在"""
        try:
            # 使用rosnode list来检查节点是否存在
            import subprocess
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True)
            return '/multi_offb_node_py' in result.stdout
        except:
            return False

    def stop_takeoff_node(self):
        """停止起飞节点"""
        rospy.loginfo("Checking if takeoff node exists...")
        
        if self.check_takeoff_node_exists():
            rospy.loginfo("Stopping takeoff node...")
            try:
                # 使用rosnode kill命令
                os.system("rosnode kill /multi_offb_node_py")
                time.sleep(0.1)
                
                # 确认节点已停止
                if not self.check_takeoff_node_exists():
                    rospy.loginfo("Takeoff node stopped successfully")
                else:
                    rospy.logwarn("Takeoff node might still be running")
            except Exception as e:
                rospy.logwarn(f"Error stopping takeoff node: {e}")
        else:
            rospy.loginfo("Takeoff node not found, may have already exited")

    def wait_for_takeoff_completion(self, timeout=60):
        """等待所有无人机完成起飞"""
        rospy.loginfo("Waiting for all UAVs to complete takeoff...")
        start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            # 检查所有无人机是否都在OFFBOARD模式并且已解锁
            ready_count = sum(1 for uav in self.uav_controllers if uav.is_ready_for_formation())
            
            if ready_count == self.num_uavs:
                rospy.loginfo("All UAVs are ready for formation flight!")
                return True
            
            # 每5秒打印一次状态
            if int(time.time() - start_time) % 5 == 0:
                rospy.loginfo(f"Ready for formation: {ready_count}/{self.num_uavs}")
                
                # 检查每架无人机的状态
                for i, uav in enumerate(self.uav_controllers):
                    if uav.connected and not uav.armed:
                        rospy.loginfo(f"UAV {i}: Connected but not armed")
                    elif uav.connected and uav.armed and uav.mode != "OFFBOARD":
                        rospy.loginfo(f"UAV {i}: Armed but not in OFFBOARD mode (current: {uav.mode})")
            
            time.sleep(1)
        
        rospy.logwarn("Not all UAVs completed takeoff within timeout")
        return False

    def initialize_formation_control(self):
        """初始化编队控制"""
        rospy.loginfo("Initializing formation control...")
        
        # 为所有无人机持续发送设定点，确保不会丢失OFFBOARD模式
        start_time = time.time()
        while time.time() - start_time < 3.0 and not rospy.is_shutdown():
            for controller in self.uav_controllers:
                if hasattr(controller.local_pose, 'pose') and hasattr(controller.local_pose.pose, 'position'):
                    controller.send_setpoint(
                        controller.local_pose.pose.position.x,
                        controller.local_pose.pose.position.y,
                        controller.local_pose.pose.position.z
                    )
            time.sleep(0.05)
        
        rospy.loginfo("Formation control initialized")
        return True

    def straight_line_formation(self, spacing=3, altitude=5, speed=0.5, direction='forward'):

        rospy.loginfo(f"Starting straight line formation in {direction} direction maintaining relative positions...")

        # 获取所有无人机的当前位置并计算相对偏移
        current_positions = []
        for controller in self.uav_controllers:
            if hasattr(controller.local_pose, 'pose') and hasattr(controller.local_pose.pose, 'position'):
                current_x = controller.local_pose.pose.position.x
                current_y = controller.local_pose.pose.position.y
                current_z = controller.local_pose.pose.position.z
                current_positions.append((current_x, current_y, current_z))
                rospy.loginfo(f"UAV {controller.uav_id} start position: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})")
            else:
                rospy.logwarn(f"UAV {controller.uav_id}: No valid position data, using default start position")
                current_positions.append((0.0, 0.0, altitude))

        # 计算编队中心点
        center_x = sum(pos[0] for pos in current_positions) / len(current_positions)
        center_y = sum(pos[1] for pos in current_positions) / len(current_positions)
        rospy.loginfo(f"Formation center: ({center_x:.2f}, {center_y:.2f})")

        # 计算每架无人机相对于中心的位置偏移
        relative_offsets = []
        for pos in current_positions:
            offset_x = pos[0] - center_x
            offset_y = pos[1] - center_y
            relative_offsets.append((offset_x, offset_y))
            rospy.loginfo(f"Relative offset: ({offset_x:.2f}, {offset_y:.2f})")

        start_time = time.time()
        rate = rospy.Rate(20)  # 20Hz

        while not rospy.is_shutdown() and self.formation_active:
            elapsed = time.time() - start_time
            travel_distance = speed * elapsed

            # 根据方向计算编队中心的位移（只移动中心点）
            formation_center_x = center_x
            formation_center_y = center_y
            if direction == 'forward':
                formation_center_y = center_y + travel_distance
            elif direction == 'backward':
                formation_center_y = center_y - travel_distance
            elif direction == 'right':
                formation_center_x = center_x + travel_distance
            elif direction == 'left':
                formation_center_x = center_x - travel_distance
            else:
                formation_center_y = center_y + travel_distance

            # 为每架无人机计算目标位置（保持相对偏移）
            for i, controller in enumerate(self.uav_controllers):
                offset_x, offset_y = relative_offsets[i]
                target_x = formation_center_x + offset_x
                target_y = formation_center_y + offset_y
                target_z = altitude

                # 发送设定点
                controller.send_setpoint(target_x, target_y, target_z)

            rate.sleep()

    def start_formation(self, direction='forward', spacing=3, altitude=5, speed=0.5):
        """开始直线编队飞行"""
        rospy.loginfo("Starting formation flight...")
        
        # 1. 等待所有无人机连接
        if not self.wait_for_all_connections(timeout=30):
            rospy.logwarn("Proceeding with available UAVs only")
        
        # 2. 等待起飞完成
        if not self.wait_for_takeoff_completion(timeout=45):
            rospy.logwarn("Some UAVs may not be ready, proceeding anyway")
        
        # 3. 停止起飞节点
        self.stop_takeoff_node()
        
        # 4. 短暂等待确保平稳过渡
        rospy.loginfo("Transitioning to formation control...")
        time.sleep(0.5)
        
        # 5. 初始化编队控制
        self.initialize_formation_control()
        
        # 6. 开始直线编队飞行
        self.formation_active = True
        
        self.formation_thread = threading.Thread(
            target=self.straight_line_formation,
            kwargs={
                'direction': direction,
                'spacing': spacing,
                'altitude': altitude,
                'speed': speed
            }
        )
        
        self.formation_thread.daemon = True
        self.formation_thread.start()
        rospy.loginfo(f"Started straight line formation in {direction} direction")
        
        return True

    def stop_formation(self):
        """停止编队飞行"""
        self.formation_active = False
        if self.formation_thread and self.formation_thread.is_alive():
            self.formation_thread.join(timeout=2.0)
        rospy.loginfo("Formation control stopped")

def main():
    # 使用命令行参数获取无人机数量
    num_uavs = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    
    # 可选参数：飞行方向、间距、高度、速度
    direction = sys.argv[2] if len(sys.argv) > 2 else 'forward'
    spacing = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    altitude = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
    speed = float(sys.argv[5]) if len(sys.argv) > 5 else 1
    
    rospy.loginfo(f"Starting multi-UAV formation control with {num_uavs} UAVs")
    rospy.loginfo(f"Direction: {direction}, Spacing: {spacing}m, Altitude: {altitude}m, Speed: {speed}m/s")
    
    # 创建编队控制器
    formation_control = MultiUAVFormationControl(num_uavs)
    
    # 设置ROS关闭时的清理函数
    def shutdown_hook():
        rospy.loginfo("Shutting down formation control...")
        formation_control.stop_formation()
    
    rospy.on_shutdown(shutdown_hook)
    
    try:
        # 等待一段时间让起飞节点完成工作
        wait_time = 15  # 默认等待15秒
        rospy.loginfo(f"Waiting {wait_time} seconds for takeoff completion...")
        rospy.sleep(wait_time)
        
        # 开始直线编队飞行
        if formation_control.start_formation(direction, spacing, altitude, speed):
            rospy.loginfo("Formation control started successfully!")
            
            # 保持运行，直到ROS关闭
            while not rospy.is_shutdown() and formation_control.formation_active:
                rospy.sleep(1)
                
        else:
            rospy.logerr("Failed to start formation control!")
            
    except KeyboardInterrupt:
        rospy.loginfo("Formation control interrupted by user")
    except Exception as e:
        rospy.logerr(f"Formation control error: {e}")
    finally:
        formation_control.stop_formation()
        rospy.loginfo("Formation control node finished")

if __name__ == "__main__":
    main()