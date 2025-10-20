#!/usr/bin/env python3

import rospy
import time
import sys
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class UAVLand:
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
        
        # 等待服务并初始化客户端
        try:
            rospy.wait_for_service(f"/{self.namespace}/mavros/cmd/arming", timeout=5)
            self.arming_client = rospy.ServiceProxy(
                f"/{self.namespace}/mavros/cmd/arming", 
                CommandBool
            )
        except rospy.ROSException:
            self.arming_client = None
            rospy.logwarn(f"UAV {self.uav_id}: Arming service not available")
        
        try:
            rospy.wait_for_service(f"/{self.namespace}/mavros/set_mode", timeout=5)
            self.set_mode_client = rospy.ServiceProxy(
                f"/{self.namespace}/mavros/set_mode", 
                SetMode
            )
        except rospy.ROSException:
            self.set_mode_client = None
            rospy.logwarn(f"UAV {self.uav_id}: Set mode service not available")
        
        rospy.loginfo(f"Initialized land controller for UAV {uav_id}")

    def state_cb(self, msg):
        """状态回调函数"""
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

    def wait_for_connection(self, timeout=10):
        """等待飞控连接"""
        rospy.loginfo(f"UAV {self.uav_id}: Waiting for connection...")
        start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if self.connected:
                rospy.loginfo(f"UAV {self.uav_id}: Connected!")
                return True
            time.sleep(0.1)
            
        rospy.logerr(f"UAV {self.uav_id}: Connection timeout!")
        return False

    def set_mode(self, mode, timeout=5):
        """设置飞行模式"""
        if self.set_mode_client is None:
            return False
            
        try:
            req = SetModeRequest()
            req.custom_mode = mode
            
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
            rospy.logwarn(f"UAV {self.uav_id}: Set mode {mode} exception: {e}")
            return False

    def land_sequence(self):
        """自动降落序列"""
        rospy.loginfo(f"UAV {self.uav_id}: Starting land sequence...")
        
        # 等待连接
        if not self.wait_for_connection(timeout=5):
            rospy.logwarn(f"UAV {self.uav_id}: Not connected, skipping")
            return False
        
        # 先切换回Position模式确保稳定
        self.set_mode("POSCTL", timeout=3)
        time.sleep(1)
        
        # 切换到AUTO.LAND模式
        land_success = False
        for mode in ["AUTO.LAND", "LAND"]:
            if self.set_mode(mode, timeout=5):
                land_success = True
                rospy.loginfo(f"UAV {self.uav_id}: {mode} mode set successfully!")
                break
        
        if not land_success:
            rospy.logerr(f"UAV {self.uav_id}: Failed to set land mode")
            return False
        
        # 等待降落完成
        rospy.loginfo(f"UAV {self.uav_id}: Waiting for land completion...")
        land_start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - land_start_time < 60:
            if not self.armed:
                rospy.loginfo(f"UAV {self.uav_id}: Land completed successfully!")
                return True
            
            # 检查是否仍在LAND模式
            if self.mode not in ["AUTO.LAND", "LAND"]:
                rospy.logwarn(f"UAV {self.uav_id}: Fell out of land mode: {self.mode}")
                # 尝试重新设置LAND模式
                for mode in ["AUTO.LAND", "LAND"]:
                    if self.set_mode(mode, timeout=2):
                        break
            
            time.sleep(0.5)
        
        rospy.logwarn(f"UAV {self.uav_id}: Land timeout after 60 seconds")
        
        # 强制解除武装
        try:
            if self.arming_client:
                disarm_req = CommandBoolRequest()
                disarm_req.value = False
                response = self.arming_client(disarm_req)
                if response.success:
                    rospy.loginfo(f"UAV {self.uav_id}: Forcibly disarmed")
                    return True
        except Exception as e:
            rospy.logwarn(f"UAV {self.uav_id}: Failed to forcibly disarm: {e}")
        
        return not self.armed


class MultiUAVLand:
    def __init__(self, num_uavs=1):
        self.num_uavs = num_uavs
        self.uav_controllers = []
        
        for i in range(num_uavs):
            controller = UAVLand(i)
            self.uav_controllers.append(controller)
        
        rospy.loginfo(f"Initialized MultiUAVLand with {num_uavs} UAV(s)")

    def start_sequential_land(self):
        """顺序启动降落"""
        if self.num_uavs == 0:
            return False
        
        results = []
        
        for i, uav in enumerate(self.uav_controllers):
            rospy.loginfo(f"Starting land for UAV {i}...")
            
            success = uav.land_sequence()
            results.append(success)
            
            if i < self.num_uavs - 1:
                rospy.loginfo(f"Waiting 3 seconds before starting next UAV...")
                time.sleep(3)
        
        success_count = sum(1 for r in results if r)
        rospy.loginfo(f"Land results: {success_count}/{self.num_uavs} successful")
        
        return success_count > 0


def get_uav_count():
    """获取无人机数量参数"""
    for arg in sys.argv:
        if arg.startswith("num_uavs:="):
            try:
                num_uavs = int(arg.split(":=")[1])
                num_uavs = max(1, min(num_uavs, 5))
                rospy.loginfo(f"Using {num_uavs} UAVs from command line")
                return num_uavs
            except:
                pass
    
    return 1


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("multi_uav_land")
    
    # 获取无人机数量
    num_uavs = get_uav_count()
    
    # 创建多无人机控制器
    controller = MultiUAVLand(num_uavs)
    
    try:
        if controller.start_sequential_land():
            rospy.loginfo("All lands completed successfully!")
        else:
            rospy.logwarn("Some lands failed!")
            
    except KeyboardInterrupt:
        rospy.loginfo("Land mission interrupted by user")
    except Exception as e:
        rospy.logerr(f"Land mission error: {e}")
    
    rospy.loginfo("Land node shutting down")