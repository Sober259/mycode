#!/usr/bin/env python3
"""
 * File: multi_uav_control.py
 * 多无人机飞行控制脚本 - 修复起飞完成检测问题
 * 适配PX4 v1.13.3
 * 专门等待无人机完成起飞后再尝试模式切换
"""

import rospy
import time
import sys
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class UAVControl:
    def __init__(self, uav_id):
        self.uav_id = uav_id
        self.namespace = f"uav{uav_id}"
        
        # 初始化当前状态
        self.current_state = State()
        self.connected = False
        self.armed = False
        self.mode = ""
        
        # 初始化订阅者
        self.state_sub = rospy.Subscriber(
            f"/{self.namespace}/mavros/state", 
            State, 
            self.state_cb
        )
        
        # 初始化发布者 - 用于位置控制
        self.local_pos_pub = rospy.Publisher(
            f"/{self.namespace}/mavros/setpoint_position/local", 
            PoseStamped, 
            queue_size=10
        )
        
        # 等待服务并初始化客户端
        try:
            rospy.wait_for_service(f"/{self.namespace}/mavros/set_mode", timeout=10)
            self.set_mode_client = rospy.ServiceProxy(
                f"/{self.namespace}/mavros/set_mode", 
                SetMode
            )
        except rospy.ROSException:
            self.set_mode_client = None
            rospy.logwarn(f"UAV {self.uav_id}: Set mode service not available")
        
        # 控制标志
        self.running = True
        self.takeoff_completed = False
        
        rospy.loginfo(f"Initialized control controller for UAV {uav_id}")

    def state_cb(self, msg):
        """状态回调函数"""
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode
        
        # 检测起飞是否完成（从AUTO.TAKEOFF切换到AUTO.LOITER）
        if self.mode == "AUTO.LOITER" and not self.takeoff_completed:
            self.takeoff_completed = True
            rospy.loginfo(f"UAV {self.uav_id}: Takeoff completed, now in LOITER mode")

    def wait_for_connection(self, timeout=15):
        """等待飞控连接"""
        rospy.loginfo(f"UAV {self.uav_id}: Waiting for connection...")
        start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if self.connected:
                rospy.loginfo(f"UAV {self.uav_id}: Connected! Current mode: {self.mode}")
                return True
            time.sleep(0.1)
            
        rospy.logerr(f"UAV {self.uav_id}: Connection timeout!")
        return False

    def wait_for_takeoff_completion(self, timeout=60):
        """等待无人机完成起飞"""
        rospy.loginfo(f"UAV {self.uav_id}: Waiting for takeoff completion...")
        start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if self.takeoff_completed:
                rospy.loginfo(f"UAV {self.uav_id}: Takeoff completed successfully")
                return True
            
            # 显示当前状态
            if time.time() - start_time > 5.0 and not self.takeoff_completed:
                rospy.loginfo(f"UAV {self.uav_id}: Current mode: {self.mode}, waiting for LOITER mode...")
            
            time.sleep(0.1)
        
        rospy.logerr(f"UAV {self.uav_id}: Takeoff completion timeout! Current mode: {self.mode}")
        return False

    def set_mode(self, mode, timeout=10):
        """设置飞行模式"""
        if self.set_mode_client is None:
            rospy.logerr(f"UAV {self.uav_id}: Set mode service not available")
            return False
            
        try:
            # 检查当前是否已经是目标模式
            if self.mode == mode:
                rospy.loginfo(f"UAV {self.uav_id}: Already in {mode} mode")
                return True
            
            req = SetModeRequest()
            req.custom_mode = mode
            
            # 发送模式设置请求
            response = self.set_mode_client(req)
            
            if not response.mode_sent:
                rospy.logwarn(f"UAV {self.uav_id}: Set mode {mode} request failed")
                return False
            
            # 等待模式切换确认
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.mode == mode:
                    rospy.loginfo(f"UAV {self.uav_id}: Successfully switched to {mode} mode")
                    return True
                
                time.sleep(0.1)
            
            rospy.logwarn(f"UAV {self.uav_id}: Mode switch to {mode} timeout, current mode: {self.mode}")
            return False
            
        except Exception as e:
            rospy.logerr(f"UAV {self.uav_id}: Set mode {mode} exception: {e}")
            return False

    def prepare_for_offboard(self, target_x, target_y, target_z, duration=5.0):
        """准备OFFBOARD模式 - 持续发布setpoint"""
        rospy.loginfo(f"UAV {self.uav_id}: Preparing for OFFBOARD mode...")
        
        start_time = time.time()
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown() and time.time() - start_time < duration:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = target_z
            pose.pose.orientation.w = 1.0
            
            self.local_pos_pub.publish(pose)
            rate.sleep()
        
        rospy.loginfo(f"UAV {self.uav_id}: OFFBOARD preparation completed")
        return True

    def fly_to_position(self, x, y, z, timeout=30, use_offboard=True):
        """飞行到指定位置"""
        if use_offboard:
            rospy.loginfo(f"UAV {self.uav_id}: Flying to position ({x}, {y}, {z}) in OFFBOARD mode...")
            target_mode = "OFFBOARD"
        else:
            rospy.loginfo(f"UAV {self.uav_id}: Flying to position ({x}, {y}, {z}) in POSCTL mode...")
            target_mode = "POSCTL"
        
        # 准备目标模式
        if use_offboard:
            self.prepare_for_offboard(x, y, z, duration=2.0)
        
        # 尝试切换到目标模式
        if not self.set_mode(target_mode, timeout=8):
            rospy.logwarn(f"UAV {self.uav_id}: Failed to set {target_mode} mode")
            
            # 如果OFFBOARD失败，尝试POSCTL作为备选
            if use_offboard:
                rospy.loginfo(f"UAV {self.uav_id}: Trying POSCTL as fallback...")
                return self.fly_to_position(x, y, z, timeout, use_offboard=False)
            else:
                return False
        
        # 持续飞行到目标位置
        start_time = time.time()
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if not self.armed:
                rospy.logerr(f"UAV {self.uav_id}: Disarmed during flight!")
                return False
            
            # 检查是否仍在目标模式
            if self.mode != target_mode:
                rospy.logwarn(f"UAV {self.uav_id}: Fell out of {target_mode} mode: {self.mode}")
                
                # 尝试恢复模式
                if not self.set_mode(target_mode, timeout=3):
                    rospy.logerr(f"UAV {self.uav_id}: Failed to recover {target_mode} mode")
                    
                    # 如果OFFBOARD失败，尝试POSCTL作为备选
                    if use_offboard:
                        rospy.loginfo(f"UAV {self.uav_id}: Trying POSCTL as fallback...")
                        return self.fly_to_position(x, y, z, timeout - (time.time() - start_time), use_offboard=False)
                    else:
                        return False
            
            # 持续发布目标位置
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            
            self.local_pos_pub.publish(pose)
            
            rate.sleep()
        
        rospy.loginfo(f"UAV {self.uav_id}: Position flight completed in {target_mode} mode")
        return True

    def fly_forward(self, distance=10.0, altitude=5.0, timeout=30, use_offboard=True):
        """向前飞行指定距离"""
        rospy.loginfo(f"UAV {self.uav_id}: Flying forward {distance}m at {altitude}m altitude...")
        
        # 计算目标位置
        target_x = self.uav_id * 5.0 + distance
        target_y = 0.0
        target_z = altitude
        
        return self.fly_to_position(target_x, target_y, target_z, timeout, use_offboard)


class MultiUAVControl:
    def __init__(self, num_uavs=1):
        self.num_uavs = num_uavs
        self.uav_controllers = []
        
        for i in range(num_uavs):
            controller = UAVControl(i)
            self.uav_controllers.append(controller)
        
        rospy.loginfo(f"Initialized MultiUAVControl with {num_uavs} UAV(s)")

    def start_control(self):
        """控制飞行"""
        if self.num_uavs == 0:
            return False
        
        results = []
        
        for i, uav in enumerate(self.uav_controllers):
            rospy.loginfo(f"Starting control for UAV {i}...")
            
            # 等待连接
            if not uav.wait_for_connection(timeout=15):
                rospy.logwarn(f"UAV {i}: Not connected, skipping")
                results.append(False)
                continue
            
            # 等待起飞完成
            if not uav.wait_for_takeoff_completion(timeout=60):
                rospy.logwarn(f"UAV {i}: Takeoff not completed, skipping")
                results.append(False)
                continue
            
            # 执行飞行
            flight_success = uav.fly_forward(distance=10.0, altitude=5.0, use_offboard=True)
            results.append(flight_success)
            
            if flight_success:
                rospy.loginfo(f"UAV {i}: Flight successful")
            
            if i < self.num_uavs - 1:
                rospy.loginfo(f"Waiting 5 seconds before starting next UAV...")
                time.sleep(5)
        
        success_count = sum(1 for r in results if r)
        rospy.loginfo(f"Control results: {success_count}/{self.num_uavs} successful")
        
        return success_count > 0


def get_uav_count():
    """获取无人机数量参数"""
    for arg in sys.argv:
        if arg.startswith("num_uavs:="):
            try:
                num_uavs = int(arg.split(":=")[1])
                num_uavs = max(1, min(num_uavs, 10))
                rospy.loginfo(f"Using {num_uavs} UAVs from command line")
                return num_uavs
            except:
                pass
    
    return 1


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("multi_uav_control")
    
    # 获取参数
    num_uavs = get_uav_count()
    
    # 创建多无人机控制器
    controller = MultiUAVControl(num_uavs)
    
    # 等待连接和起飞完成将在start_control中处理
    try:
        if controller.start_control():
            rospy.loginfo("All control missions completed successfully!")
        else:
            rospy.logwarn("Some control missions failed!")
            
    except KeyboardInterrupt:
        rospy.loginfo("Control mission interrupted by user")
    except Exception as e:
        rospy.logerr(f"Control mission error: {e}")
    
    # 保持节点运行
    rospy.loginfo("Control node finished. Press Ctrl+C to exit.")
    rospy.spin()