#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import yaml
import os
import moveit_commander
from std_srvs.srv import Trigger, SetBool, SetBoolResponse, TriggerResponse
from geometry_msgs.msg import Pose
# 【新增】控制器切换服务消息类型
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

class RobotManager:
    def __init__(self):
        rospy.init_node('vs_robot_manager')
        
        # 1. 初始化 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.group_name = rospy.get_param("~group_name", "manipulator") 
        
        try:
            self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
            self.move_group.set_max_velocity_scaling_factor(0.1) 
            self.move_group.set_max_acceleration_scaling_factor(0.3)
            rospy.loginfo(f"[Manager] MoveGroup '{self.group_name}' connected.")
        except Exception as e:
            rospy.logerr(f"[Manager] Failed to connect to MoveGroup: {e}")
            sys.exit(1)
        
        # 2. 状态文件路径
        self.config_path = rospy.get_param("~config_path", "")
        if self.config_path == "":
            base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.config_path = os.path.join(base_path, 'param', 'joint_angels','saved_poses.yaml')
        
        self.saved_poses = {}
        self.load_poses()

        # 3. 连接外部服务
        self.srv_bridge_enable = rospy.ServiceProxy('/vs_bridge/enable', SetBool)
        self.srv_vs_control = rospy.ServiceProxy('/vs/enable_control', SetBool)
        self.srv_dvs_snapshot = rospy.ServiceProxy('/vs_dvs/snapshot', Trigger)
        
        # 【新增】控制器管理器服务
        rospy.loginfo("[Manager] Waiting for controller_manager switch_controller service...")
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=5.0)
            self.srv_switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            rospy.loginfo("[Manager] Controller Manager Connected.")
        except Exception as e:
            rospy.logerr(f"[Manager] Failed to connect to Controller Manager: {e}")
            self.srv_switch_controller = None

        # 定义控制器名称 (根据你的 log 修改)
        self.traj_controller = 'pos_joint_traj_controller' # 用于 MoveIt (H键/数字键)
        self.vel_controller  = 'twist_controller'          # 用于 视觉伺服/键盘 (B键/V键)

        # 4. 提供服务
        rospy.Service('/vs_manager/servo_toggle', SetBool, self.cb_servo_toggle)   # V键
        rospy.Service('/vs_manager/manual_toggle', SetBool, self.cb_manual_toggle) # B键
        rospy.Service('/vs_manager/save_target', Trigger, self.cb_save_target)     # M键
        
        rospy.Service('/vs_manager/save_pose_slot', Trigger, self.cb_save_pose_slot) # R + 1-4
        rospy.Service('/vs_manager/move_to_slot', Trigger, self.cb_move_to_slot)     # 1-4
        rospy.Service('/vs_manager/home', Trigger, self.cb_go_home)                  # H键

        self.is_servoing = False
        rospy.loginfo(f"[Manager] Ready. Poses will be saved to: {self.config_path}")

    # --- 辅助功能：自动切换控制器 ---
    def switch_to_trajectory_mode(self):
        """ 切换到 MoveIt 模式：开启 trajectory, 关闭 twist """
        if self.srv_switch_controller is None: return False
        
        req = SwitchControllerRequest()
        req.start_controllers = [self.traj_controller]
        req.stop_controllers  = [self.vel_controller]
        req.strictness = SwitchControllerRequest.BEST_EFFORT
        
        try:
            res = self.srv_switch_controller(req)
            if res.ok:
                rospy.loginfo(f"[Manager] Switched to TRAJECTORY mode ({self.traj_controller} RUNNING)")
                return True
            else:
                rospy.logwarn(f"[Manager] Failed to switch to TRAJECTORY mode.")
                return False
        except Exception as e:
            rospy.logerr(f"Switch Controller Error: {e}")
            return False

    def switch_to_velocity_mode(self):
        """ 切换到 速度控制 模式：开启 twist, 关闭 trajectory """
        if self.srv_switch_controller is None: return False
        
        req = SwitchControllerRequest()
        req.start_controllers = [self.vel_controller]
        req.stop_controllers  = [self.traj_controller]
        req.strictness = SwitchControllerRequest.BEST_EFFORT
        
        try:
            res = self.srv_switch_controller(req)
            if res.ok:
                rospy.loginfo(f"[Manager] Switched to VELOCITY mode ({self.vel_controller} RUNNING)")
                return True
            else:
                rospy.logwarn(f"[Manager] Failed to switch to VELOCITY mode.")
                return False
        except Exception as e:
            rospy.logerr(f"Switch Controller Error: {e}")
            return False

    # --- 基础功能 ---
    def load_poses(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    self.saved_poses = yaml.safe_load(f) or {}
                rospy.loginfo(f"[Manager] Loaded {len(self.saved_poses)} poses from disk.")
            else:
                self.saved_poses = {}
        except Exception as e:
            rospy.logerr(f"[Manager] Failed to load poses: {e}")

    def save_poses_to_disk(self):
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, 'w') as f:
                yaml.dump(self.saved_poses, f)
            rospy.loginfo("[Manager] Poses saved to disk.")
        except Exception as e:
            rospy.logerr(f"[Manager] Failed to save poses: {e}")

    # --- 回调逻辑 ---

    def cb_go_home(self, req):
        """ H键逻辑：回到 Home 姿态 """
        if self.is_servoing:
            return TriggerResponse(False, "DANGER: Visual Servoing is ON. Press 'SPACE' first.")

        # 【自动切换】准备执行 MoveIt 动作前，切换到 trajectory 控制器
        if not self.switch_to_trajectory_mode():
            return TriggerResponse(False, "Controller Switch Failed! (Check terminal)")

        target_key = 'home'
        if target_key not in self.saved_poses:
            if 'slot_1' in self.saved_poses:
                rospy.logwarn("[Manager] 'home' pose not found, using 'slot_1' instead.")
                target_key = 'slot_1'
            else:
                return TriggerResponse(False, "No 'home' or 'slot_1' pose defined in saved_poses.yaml!")
        
        target_joints = self.saved_poses[target_key]
        rospy.loginfo(f"[Manager] Moving to {target_key}...")
        
        success = self.move_group.go(target_joints, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            return TriggerResponse(True, "Robot is at HOME.")
        else:
            return TriggerResponse(False, "MoveIt Planning Failed.")

    def cb_manual_toggle(self, req):
        """ B键逻辑：手动模式 """
        res = SetBoolResponse()
        if req.data: self.move_group.stop()

        # 【自动切换】如果是开启手动模式，切换到 velocity 控制器
        if req.data:
            self.switch_to_velocity_mode()
        # 如果是关闭手动模式(req.data=False)，通常是为了停止，我们可以不强制切回 trajectory，
        # 或者为了安全，保持现状，等到下次按H键再切回。这里保持现状。

        try:
            # 关算法
            try:
                self.srv_vs_control.wait_for_service(0.5)
                self.srv_vs_control(False)
            except: pass

            # 开桥接
            self.srv_bridge_enable.wait_for_service(1.0)
            bridge_res = self.srv_bridge_enable(req.data)
            
            res.success = bridge_res.success
            mode_str = "MANUAL MODE (WASD Enabled)" if req.data else "LOCKED (Bridge OFF)"
            res.message = f"Manager: {mode_str}"
            rospy.loginfo(res.message)
            self.is_servoing = False

        except Exception as e:
            res.success = False
            res.message = f"Manual toggle failed: {e}"
            rospy.logerr(res.message)
        return res

    def cb_servo_toggle(self, req):
        """ V键逻辑：自动伺服 """
        res = SetBoolResponse()
        if req.data: self.move_group.stop()
        
        # 【自动切换】如果是开启伺服，切换到 velocity 控制器
        if req.data:
            self.switch_to_velocity_mode()

        try:
            # 开桥接
            self.srv_bridge_enable.wait_for_service(1.0)
            bridge_res = self.srv_bridge_enable(req.data)

            # 开算法
            vs_msg = "Skipped"
            try:
                self.srv_vs_control.wait_for_service(1.0)
                self.srv_vs_control(req.data)
                vs_msg = "Algo ON" if req.data else "Algo OFF"
            except:
                vs_msg = "Algo NOT FOUND"

            self.is_servoing = req.data
            res.success = bridge_res.success
            state = "AUTO SERVO" if req.data else "STOPPED"
            res.message = f"Manager: {state} | {bridge_res.message} | {vs_msg}"
            rospy.loginfo(res.message)
        
        except Exception as e:
            res.success = False
            res.message = f"Servo toggle failed: {e}"
            rospy.logerr(res.message)
        return res

    def cb_save_target(self, req):
        """ M键逻辑：保存目标 """
        # 这个动作是纯读取，不需要切换控制器
        res = TriggerResponse()
        try:
            self.srv_dvs_snapshot.wait_for_service(timeout=1.0)
            dvs_res = self.srv_dvs_snapshot()
            if not dvs_res.success:
                return TriggerResponse(False, f"Snapshot Failed: {dvs_res.message}")

            current_joints = self.move_group.get_current_joint_values()
            p = self.move_group.get_current_pose().pose
            pose_dict = {'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                         'qx': p.orientation.x, 'qy': p.orientation.y, 
                         'qz': p.orientation.z, 'qw': p.orientation.w}
            
            self.saved_poses['target_reference'] = {
                'joints': current_joints, 'pose': pose_dict, 'description': "Saved by User"
            }
            self.save_poses_to_disk()
            res.success = True
            res.message = "Target Saved."
            rospy.loginfo(res.message)
        except Exception as e:
            res.success = False
            res.message = f"Save failed: {e}"
            rospy.logerr(res.message)
        return res

    def cb_save_pose_slot(self, req):
        """ 保存到槽位 (R + 数字键) """
        slot_id = rospy.get_param("/vs_manager/current_slot_id", 1)
        try:
            joints = self.move_group.get_current_joint_values()
            key = f"slot_{slot_id}"
            self.saved_poses[key] = joints
            self.save_poses_to_disk()
            
            msg = f"Pose saved to Slot {slot_id}"
            rospy.loginfo(msg)
            return TriggerResponse(True, msg)
        except Exception as e:
            return TriggerResponse(False, f"Save failed: {e}")

    def cb_move_to_slot(self, req):
        """ 移动到槽位 (数字键) """
        if self.is_servoing:
            return TriggerResponse(False, "DANGER: Visual Servoing is ON. Press 'SPACE' first.")

        # 【自动切换】MoveIt 动作，确保是 trajectory 模式
        if not self.switch_to_trajectory_mode():
            return TriggerResponse(False, "Controller Switch Failed!")

        slot_id = rospy.get_param("/vs_manager/current_slot_id", 1)
        key = f"slot_{slot_id}"
        
        if key not in self.saved_poses:
            return TriggerResponse(False, f"Slot {slot_id} is empty. Record it first (R + {slot_id}).")
            
        target_joints = self.saved_poses[key]
        rospy.loginfo(f"[Manager] Moving to Slot {slot_id}...")
        
        success = self.move_group.go(target_joints, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            return TriggerResponse(True, f"Moved to Slot {slot_id}")
        else:
            return TriggerResponse(False, "MoveIt Planning/Execution Failed")

if __name__ == "__main__":
    RobotManager()
    rospy.spin()