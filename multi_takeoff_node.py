#!/usr/bin/env python3
"""
 * File: multi_takeoff_node.py
 * 多无人机自动起飞控制节点 - 修复高度、多机控制和等待时间问题
"""

import rospy
import threading
import time
import sys
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class UAVTakeoffControl:
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
        
        # 控制标志
        self.running = True
        self.takeoff_complete = False
        
        rospy.loginfo(f"Initialized takeoff controller for UAV {uav_id}")

    def state_cb(self, msg):
        """状态回调函数"""
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

    def wait_for_connection(self, timeout=15):
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

    def set_mode(self, mode, timeout=3):
        """设置飞行模式"""
        if self.set_mode_client is None:
            return False
            
        try:
            req = SetModeRequest()
            req.custom_mode = mode
            
            response = self.set_mode_client(req)
            
            if not response.mode_sent:
                return False
            
            # 等待模式切换确认
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.mode == mode:
                    rospy.loginfo(f"UAV {self.uav_id}: Successfully switched to {mode} mode")
                    return True
                time.sleep(0.1)
            
            return False
            
        except Exception as e:
            rospy.logwarn(f"UAV {self.uav_id}: Set mode {mode} exception: {e}")
            return False

    def arm(self, timeout=3):
        """解锁无人机"""
        if self.arming_client is None:
            return False
            
        try:
            if self.armed:
                return True
            
            req = CommandBoolRequest()
            req.value = True
            
            response = self.arming_client(req)
            
            if not response.success:
                return False
            
            # 等待解锁完成
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.armed:
                    rospy.loginfo(f"UAV {self.uav_id}: Successfully armed")
                    return True
                time.sleep(0.1)
            
            return False
            
        except Exception as e:
            rospy.logwarn(f"UAV {self.uav_id}: Arming exception: {e}")
            return False

    def set_takeoff_altitude(self, altitude=5.0):
        """设置起飞高度参数"""
        try:
            rospy.wait_for_service(f"/{self.namespace}/mavros/param/set", timeout=3)
            from mavros_msgs.srv import ParamSet, ParamSetRequest
            param_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/param/set", ParamSet)
            
            req = ParamSetRequest()
            req.param_id = "MIS_TAKEOFF_ALT"
            req.value.real = altitude
            
            response = param_client(req)
            if response.success:
                rospy.loginfo(f"UAV {self.uav_id}: Set takeoff altitude to {altitude}m")
                return True
            else:
                rospy.logwarn(f"UAV {self.uav_id}: Failed to set takeoff altitude")
                return False
        except:
            rospy.logwarn(f"UAV {self.uav_id}: Could not set takeoff altitude parameter")
            return False

    def takeoff_sequence(self, target_altitude=5.0):
        """自动起飞序列"""
        rospy.loginfo(f"UAV {self.uav_id}: Starting takeoff sequence to {target_altitude}m...")
        
        # 1. 等待连接
        if not self.wait_for_connection(timeout=10):
            return False
        
        # 2. 设置起飞高度
        self.set_takeoff_altitude(target_altitude)
        time.sleep(1)
        
        # 3. 尝试在当前模式下解锁
        rospy.loginfo(f"UAV {self.uav_id}: Current mode: {self.mode}")
        
        if not self.arm(timeout=3):
            rospy.logwarn(f"UAV {self.uav_id}: Failed to arm in current mode {self.mode}")
            
            # 尝试切换到可以解锁的模式
            for mode in ["POSCTL", "ALTCTL", "MANUAL"]:
                if self.set_mode(mode, timeout=2):
                    time.sleep(0.5)
                    if self.arm(timeout=3):
                        break
                    else:
                        rospy.logwarn(f"UAV {self.uav_id}: Failed to arm in {mode} mode")
        
        if not self.armed:
            rospy.logerr(f"UAV {self.uav_id}: Failed to arm in any mode")
            return False
        
        # 4. 切换到AUTO.TAKEOFF模式
        takeoff_success = False
        for mode in ["AUTO.TAKEOFF", "TAKEOFF"]:
            if self.set_mode(mode, timeout=3):
                takeoff_success = True
                break
        
        if not takeoff_success:
            rospy.logerr(f"UAV {self.uav_id}: Failed to set takeoff mode")
            return False
        
        # 5. 等待起飞完成
        rospy.loginfo(f"UAV {self.uav_id}: Waiting for takeoff completion...")
        takeoff_start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - takeoff_start_time < 25:
            if self.mode not in ["AUTO.TAKEOFF", "TAKEOFF"]:
                rospy.loginfo(f"UAV {self.uav_id}: Takeoff completed! Current mode: {self.mode}")
                self.takeoff_complete = True
                return True
            
            if not self.armed:
                rospy.logerr(f"UAV {self.uav_id}: Disarmed during takeoff!")
                return False
            
            time.sleep(0.5)
        
        rospy.logwarn(f"UAV {self.uav_id}: Takeoff timeout")
        return False

    def land_sequence(self):
        """自动降落序列"""
        rospy.loginfo(f"UAV {self.uav_id}: Starting land sequence...")
        
        for mode in ["AUTO.LAND", "LAND"]:
            if self.set_mode(mode, timeout=3):
                break
        
        # 等待降落完成
        land_start_time = time.time()
        
        while not rospy.is_shutdown() and time.time() - land_start_time < 30:
            if not self.armed:
                rospy.loginfo(f"UAV {self.uav_id}: Land completed successfully!")
                return True
            time.sleep(0.5)
        
        # 强制解除武装
        try:
            if self.arming_client:
                disarm_req = CommandBoolRequest()
                disarm_req.value = False
                self.arming_client(disarm_req)
                rospy.loginfo(f"UAV {self.uav_id}: Forcibly disarmed")
                return True
        except:
            pass
        
        return not self.armed

    def run_takeoff_test(self):
        """运行起飞测试"""
        rospy.loginfo(f"UAV {self.uav_id}: Starting takeoff test...")
        
        if self.takeoff_sequence(target_altitude=5.0):
            rospy.loginfo(f"UAV {self.uav_id}: Takeoff successful! Hovering for 5 seconds...")
            
            # 悬停5秒
            hover_start = time.time()
            while not rospy.is_shutdown() and time.time() - hover_start < 5:
                if not self.armed:
                    return False
                time.sleep(0.5)
            
            if self.land_sequence():
                rospy.loginfo(f"UAV {self.uav_id}: Mission completed successfully!")
                return True
            else:
                rospy.logerr(f"UAV {self.uav_id}: Land failed!")
                return False
        else:
            rospy.logerr(f"UAV {self.uav_id}: Takeoff failed!")
            return False


class MultiUAVTakeoffControl:
    def __init__(self, num_uavs=1):
        self.num_uavs = num_uavs
        self.uav_controllers = []
        
        for i in range(num_uavs):
            controller = UAVTakeoffControl(i)
            self.uav_controllers.append(controller)
        
        rospy.loginfo(f"Initialized MultiUAVTakeoffControl with {num_uavs} UAV(s)")

    def start_parallel_takeoff(self):
        """并行启动起飞测试"""
        if self.num_uavs == 0:
            return False
        
        # 使用线程并行执行
        threads = []
        results = [False] * self.num_uavs
        
        for i, uav in enumerate(self.uav_controllers):
            thread = threading.Thread(
                target=lambda idx=i, u=uav: self._run_uav_test(idx, u, results),
                name=f"uav_{i}_thread"
            )
            threads.append(thread)
        
        # 启动所有线程
        for thread in threads:
            thread.start()
        
        # 等待所有线程完成
        for thread in threads:
            thread.join(timeout=120)  # 2分钟超时
        
        success_count = sum(1 for r in results if r)
        rospy.loginfo(f"Takeoff test results: {success_count}/{self.num_uavs} successful")
        
        return success_count > 0

    def _run_uav_test(self, idx, uav, results):
        """运行单个无人机测试"""
        try:
            results[idx] = uav.run_takeoff_test()
        except Exception as e:
            rospy.logerr(f"UAV {idx}: Test failed with exception: {e}")
            results[idx] = False


def get_uav_count():
    """获取无人机数量参数 - 从命令行参数或ROS参数获取"""
    # 首先尝试从命令行参数获取
    for arg in sys.argv:
        if arg.startswith("num_uavs:="):
            try:
                num_uavs = int(arg.split(":=")[1])
                num_uavs = max(1, min(num_uavs, 10))
                rospy.loginfo(f"Using {num_uavs} UAVs from command line argument")
                return num_uavs
            except:
                pass
    
    # 然后尝试从ROS参数获取
    try:
        if rospy.has_param("num_uavs"):
            num_uavs = rospy.get_param("num_uavs", 1)
            num_uavs = max(1, min(num_uavs, 10))
            rospy.loginfo(f"Using {num_uavs} UAVs from ROS parameter")
            return num_uavs
    except:
        pass
    
    # 最后使用默认值
    rospy.loginfo("Using default: 1 UAV")
    return 1


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("multi_takeoff_control")
    
    # 获取无人机数量
    num_uavs = get_uav_count()
    
    # 缩短等待时间
    rospy.loginfo(f"Waiting for simulation to be ready (10 seconds) for {num_uavs} UAVs...")
    time.sleep(10)
    
    # 创建多无人机控制器
    controller = MultiUAVTakeoffControl(num_uavs)
    
    try:
        if controller.start_parallel_takeoff():
            rospy.loginfo("All takeoff tests completed successfully!")
        else:
            rospy.logwarn("Some takeoff tests failed!")
            
    except KeyboardInterrupt:
        rospy.loginfo("Takeoff test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Takeoff test error: {e}")
    
    rospy.loginfo("Multi-UAV takeoff control node shutting down")