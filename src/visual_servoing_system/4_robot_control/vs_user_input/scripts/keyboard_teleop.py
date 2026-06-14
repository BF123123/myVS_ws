#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, select, termios, tty
import rospkg
import yaml
import os
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool

msg = """
---------------------------------------
Visual Servoing Keyboard Controller
---------------------------------------
[Control Modes]
   B: Toggle MANUAL Mode (Enable WASD)
   V: Toggle AUTO SERVO (Enable Algo)
   SPACE: STOP ALL (Emergency Stop)

[Configuration (Input Mode)]
   G: Set Depth Strategy (Input name: hybrid, desired_only, constant)
   F: Set Algorithm      (Input name: dvs, ibvs)
   (Changes are saved to .yaml automatically)

[Pose Recording]
   R:   Record Mode
   1-4: Move/Save Slot
   H:   Go Home

[Moving - Manual Mode]
   Q W E
   A S D
   I J K L U O (Rotation)

CTRL-C to quit
"""

moveBindings = {
    'w':(0,0,0.05,0,0,0),  's':(0,0,-0.05,0,0,0), 
    'a':(-0.05,0,0,0,0,0), 'd':(0.05,0,0,0,0,0),  
    'q':(0,-0.05,0,0,0,0), 'e':(0,0.05,0,0,0,0),  
    'i':(0,0,0,0,-0.1,0),  'k':(0,0,0,0,0.1,0),   
    'j':(0,0,0,0,0,0.1),   'l':(0,0,0,0,0,-0.1),  
    'u':(0,0,0,0.1,0,0),   'o':(0,0,0,-0.1,0,0),  
}

class ConfigManager:
    def __init__(self):
        # 1. 动态定位 YAML 文件路径
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path('vs_control')
            self.yaml_path = os.path.join(pkg_path, 'config', 'vs_control_params.yaml')
            rospy.loginfo(f"Config file loaded: {self.yaml_path}")
        except Exception as e:
            rospy.logerr(f"Could not find vs_control package: {e}")
            self.yaml_path = None

    def update_yaml(self, param_name, new_value):
        """ 读取 -> 修改 -> 保存 YAML """
        if not self.yaml_path or not os.path.exists(self.yaml_path):
            rospy.logerr("YAML path invalid, cannot save to disk.")
            return

        try:
            # 读取
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            
            # 修改
            # 注意：这里假设 yaml 结构是扁平的或者 param_name 在根目录下
            # 如果您的 yaml 有层级，需要对应处理
            if param_name in data:
                old_val = data[param_name]
                data[param_name] = new_value
                rospy.loginfo(f"[Disk] Updated {param_name}: {old_val} -> {new_value}")
            else:
                rospy.logwarn(f"[Disk] Key '{param_name}' not found in YAML, adding it.")
                data[param_name] = new_value

            # 保存 (使用 safe_dump 保持格式)
            with open(self.yaml_path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)
                
        except Exception as e:
            rospy.logerr(f"Failed to update YAML: {e}")

# 全局设置变量，用于模式切换
settings = None

def getKey():
    """ Raw Mode: 获取单个按键，不回显，无需回车 """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def get_input_string(prompt):
    """ Cooked Mode: 恢复终端设置，获取字符串输入，需回车 """
    # 1. 恢复终端设置 (Cooked Mode)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    # 2. 获取输入
    try:
        user_input = input(f"\n>> {prompt}")
    except EOFError:
        user_input = ""
    
    # 3. 再次切换回 Raw Mode (由主循环 getKey 处理，这里只需返回即可，
    # 但为了逻辑闭环，调用结束时主循环的 getKey 会再次负责 setraw)
    return user_input.strip()

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('vs_keyboard_teleop')
    
    # 初始化配置管理器
    config_mgr = ConfigManager()

    pub = rospy.Publisher('/vs/camera_velocity_raw', Twist, queue_size=1)
    
    # 服务连接
    srv_save_target = rospy.ServiceProxy('/vs_manager/save_target', Trigger)
    srv_servo_toggle = rospy.ServiceProxy('/vs_manager/servo_toggle', SetBool)
    srv_manual_toggle = rospy.ServiceProxy('/vs_manager/manual_toggle', SetBool)
    srv_save_slot = rospy.ServiceProxy('/vs_manager/save_pose_slot', Trigger)
    srv_move_slot = rospy.ServiceProxy('/vs_manager/move_to_slot', Trigger)
    srv_home = rospy.ServiceProxy('/vs_manager/home', Trigger)
    
    # 重载服务
    srv_reload_config = rospy.ServiceProxy('/vs/reload_config', Trigger)

    x = 0; y = 0; z = 0
    thx = 0; thy = 0; thz = 0
    in_auto_mode = False 
    record_mode_pending = False

    try:
        print(msg)
        while(1):
            key = getKey()
            
            # --- 1. 运动控制 ---
            if key in moveBindings.keys():
                if in_auto_mode:
                    print(">> [OVERRIDE] Key pressed! Switching to MANUAL mode.")
                    try:
                        srv_servo_toggle(False)
                        srv_manual_toggle(True)
                    except: pass
                    in_auto_mode = False

                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                thx = moveBindings[key][3]
                thy = moveBindings[key][4]
                thz = moveBindings[key][5]
                
                twist = Twist()
                twist.linear.x = x; twist.linear.y = y; twist.linear.z = z
                twist.angular.x = thx; twist.angular.y = thy; twist.angular.z = thz
                pub.publish(twist)

            # --- 2. 配置修改 (G键 - 深度策略) ---
            elif key == 'g' or key == 'G':
                # 进入输入模式
                strat_name = get_input_string("Enter Depth Strategy (hybrid/desired_only/constant): ")
                
                if strat_name:
                    # A. 更新 Parameter Server (供 C++ 读取)
                    # 注意: 节点名需与 launch 文件一致 (vs_controller)
                    param_name = "/vs_controller/depth_strategy"
                    rospy.set_param(param_name, strat_name)
                    
                    # B. 更新本地 YAML (供下次启动)
                    config_mgr.update_yaml("depth_strategy", strat_name)
                    
                    # C. 触发重载
                    try:
                        resp = srv_reload_config()
                        print(f"   [Reload] {resp.message}")
                    except Exception as e:
                        print(f"   [Error] Reload failed: {e}")
                else:
                    print("   [Cancel] Empty input.")

            # --- 3. 配置修改 (F键 - 算法类型) ---
            elif key == 'f' or key == 'F':
                # 进入输入模式
                algo_name = get_input_string("Enter Algorithm (dvs/ibvs): ")
                
                if algo_name:
                    param_name = "/vs_controller/algorithm_type"
                    rospy.set_param(param_name, algo_name)
                    
                    config_mgr.update_yaml("algorithm_type", algo_name)
                    
                    try:
                        resp = srv_reload_config()
                        print(f"   [Reload] {resp.message}")
                    except Exception as e:
                        print(f"   [Error] Reload failed: {e}")
                else:
                    print("   [Cancel] Empty input.")

            # --- 其他按键 (1-4, R, H, B, V, M) ---
            elif key in ['1', '2', '3', '4']:
                slot_id = int(key)
                rospy.set_param("/vs_manager/current_slot_id", slot_id)
                if record_mode_pending:
                    print(f">> [CMD] Saving to Slot {slot_id}...")
                    try: srv_save_slot()
                    except: pass
                    record_mode_pending = False
                else:
                    print(f">> [CMD] Moving to Slot {slot_id}...")
                    try: srv_move_slot()
                    except: pass
            
            elif key == 'r' or key == 'R':
                record_mode_pending = True
                print(">> [RECORD] Press 1-4 to save.")

            elif key == 'h' or key == 'H':
                print(">> [HOME]")
                try: srv_home()
                except: pass

            elif key == 'b' or key == 'B':
                print(">> [MANUAL]")
                in_auto_mode = False
                try: srv_manual_toggle(True)
                except: pass
            
            elif key == 'v' or key == 'V':
                print(">> [AUTO SERVO]")
                in_auto_mode = True
                try: srv_servo_toggle(True)
                except: pass

            elif key == 'm' or key == 'M':
                print(">> [SAVE TARGET]")
                try: srv_save_target()
                except: pass

            elif key == ' ':
                print(">> STOP ALL")
                in_auto_mode = False
                pub.publish(Twist())
                try: 
                    srv_manual_toggle(False)
                    srv_servo_toggle(False)
                except: pass

            elif key == '\x03': # Ctrl-C
                break
            
            else:
                if key == '' and not in_auto_mode:
                    pub.publish(Twist())

    except Exception as e:
        print(e)

    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)