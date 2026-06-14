#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import yaml
import os
import re
import tkinter as tk
from tkinter import ttk, messagebox
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool

class ConfigManager:
    def __init__(self):
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path('vs_control')
            self.yaml_path = os.path.join(pkg_path, 'config', 'vs_control_params.yaml')

            user_input_path = rospack.get_path('vs_user_input')
            self.launch_path = os.path.join(user_input_path, 'launch', 'vs_experiment_start.launch')
            rospy.loginfo(f"Config file loaded: {self.yaml_path}")
        except Exception as e:
            rospy.logerr(f"Could not find packages: {e}")
            self.yaml_path = None
            self.launch_path = None

    def update_launch_default(self, arg_name, new_value):
        if not self.launch_path or not os.path.exists(self.launch_path): return
        try:
            with open(self.launch_path, 'r') as f: content = f.read()
            new_content = re.sub(
                rf'(<arg\s+name="{arg_name}"\s+default=")[^"]*(")',
                r'\g<1>' + new_value + r'\g<2>', content, count=1
            )
            with open(self.launch_path, 'w') as f: f.write(new_content)
        except Exception as e:
            rospy.logerr(f"Failed to update launch file: {e}")

    def update_yaml(self, param_name, new_value):
        if not self.yaml_path or not os.path.exists(self.yaml_path): return
        try:
            with open(self.yaml_path, 'r') as f: data = yaml.safe_load(f) or {}
            data[param_name] = new_value
            with open(self.yaml_path, 'w') as f: yaml.safe_dump(data, f, default_flow_style=False)
        except Exception as e:
            rospy.logerr(f"Failed to update YAML: {e}")


class VSDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Visual Servoing Control Dashboard")
        self.root.geometry("750x650")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # ROS Setup
        rospy.init_node('vs_gui_teleop', anonymous=True, disable_signals=True)
        self.config_mgr = ConfigManager()
        self.pub = rospy.Publisher('/vs/camera_velocity_raw', Twist, queue_size=1)
        
        # Service Proxies
        self.srv_save_target = rospy.ServiceProxy('/vs_manager/save_target', Trigger)
        self.srv_servo_toggle = rospy.ServiceProxy('/vs_manager/servo_toggle', SetBool)
        self.srv_manual_toggle = rospy.ServiceProxy('/vs_manager/manual_toggle', SetBool)
        self.srv_save_slot = rospy.ServiceProxy('/vs_manager/save_pose_slot', Trigger)
        self.srv_move_slot = rospy.ServiceProxy('/vs_manager/move_to_slot', Trigger)
        self.srv_home = rospy.ServiceProxy('/vs_manager/home', Trigger)
        self.srv_reload_config = rospy.ServiceProxy('/vs/reload_config', Trigger)
        self.srv_direct_enable = rospy.ServiceProxy('/vs/enable_control', SetBool)
        
        self.in_auto_mode = False
        self.current_twist = Twist()

        self.create_widgets()
        
        # 启动后台发布循环 (完美模仿原键盘节点的 while(1) 机制)
        self.publish_move_loop()

    def create_widgets(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # === 1. TOP: Status & Emergency ===
        frame_top = tk.Frame(self.root, pady=10)
        frame_top.pack(fill=tk.X, padx=20)
        
        self.lbl_status = tk.Label(frame_top, text="Mode: MANUAL", font=("Arial", 16, "bold"), fg="blue")
        self.lbl_status.pack(side=tk.LEFT)
        
        btn_stop = tk.Button(frame_top, text="EMERGENCY STOP (SPACE)", font=("Arial", 14, "bold"), bg="red", fg="white", command=self.stop_all)
        btn_stop.pack(side=tk.RIGHT)
        self.root.bind('<space>', lambda e: self.stop_all())

        # === 2. SETTINGS PANEL ===
        frame_settings = ttk.LabelFrame(self.root, text=" System Configuration ")
        frame_settings.pack(fill=tk.X, padx=20, pady=10, ipadx=10, ipady=10)
        
        self.var_cam = tk.StringVar(value=rospy.get_param("/vs_controller/primary_camera", "basler"))
        self.var_algo = tk.StringVar(value=rospy.get_param("/vs_controller/algorithm_type", "defocused"))
        self.var_depth = tk.StringVar(value=rospy.get_param("/vs_controller/depth_strategy", "constant"))

        ttk.Label(frame_settings, text="Primary Camera:").grid(row=0, column=0, sticky=tk.E, padx=5, pady=5)
        cb_cam = ttk.Combobox(frame_settings, textvariable=self.var_cam, values=["l515", "basler"], state="readonly")
        cb_cam.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(frame_settings, text="Algorithm:").grid(row=0, column=2, sticky=tk.E, padx=5, pady=5)
        cb_algo = ttk.Combobox(frame_settings, textvariable=self.var_algo, values=["dvs", "defocused", "depth", "defocused_RAL"], state="readonly")
        cb_algo.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(frame_settings, text="Depth Strategy:").grid(row=0, column=4, sticky=tk.E, padx=5, pady=5)
        cb_depth = ttk.Combobox(frame_settings, textvariable=self.var_depth, values=["realtime", "constant", "desired_only"], state="readonly")
        cb_depth.grid(row=0, column=5, padx=5, pady=5)

        btn_apply = ttk.Button(frame_settings, text="Apply & Reload", command=self.apply_settings)
        btn_apply.grid(row=1, column=0, columnspan=6, pady=10)

        # === 3. MAIN CONTROL & WAYPOINTS ===
        frame_mid = tk.Frame(self.root)
        frame_mid.pack(fill=tk.BOTH, expand=True, padx=20)

        # Left: Waypoints
        frame_wp = ttk.LabelFrame(frame_mid, text=" Waypoints & Tasks ")
        frame_wp.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        btn_save_target = tk.Button(frame_wp, text="[M] SAVE TARGET IMAGE", bg="lightgreen", font=("Arial", 12, "bold"), command=self.cmd_save_target)
        btn_save_target.pack(fill=tk.X, padx=10, pady=10)
        
        btn_servo = tk.Button(frame_wp, text="[V] START AUTO SERVO", bg="cyan", font=("Arial", 12, "bold"), command=self.cmd_auto_servo)
        btn_servo.pack(fill=tk.X, padx=10, pady=10)

        ttk.Separator(frame_wp, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        for i in range(1, 5):
            f = tk.Frame(frame_wp)
            f.pack(fill=tk.X, padx=10, pady=2)
            ttk.Button(f, text=f"Move Slot {i}", command=lambda idx=i: self.cmd_move_slot(idx)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
            ttk.Button(f, text=f"Save Slot {i}", command=lambda idx=i: self.cmd_save_slot(idx)).pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=2)

        ttk.Button(frame_wp, text="[H] Home Robot", command=self.cmd_home).pack(fill=tk.X, padx=10, pady=10)

        # Right: Teleop
        frame_teleop = ttk.LabelFrame(frame_mid, text=" Teleop (Hold to Move) ")
        frame_teleop.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        lbl_tip = tk.Label(frame_teleop, text="Press and Hold buttons to move camera", fg="gray")
        lbl_tip.pack(pady=5)

        # XYZ Grid
        frame_xyz = tk.Frame(frame_teleop)
        frame_xyz.pack(pady=10)
        self.make_move_btn(frame_xyz, "Forward (+Z)", 0, 0, 0.04, 0, 0, 0).grid(row=0, column=1)
        self.make_move_btn(frame_xyz, "Left (-X)", -0.04, 0, 0, 0, 0, 0).grid(row=1, column=0)
        self.make_move_btn(frame_xyz, "Backward (-Z)", 0, 0, -0.04, 0, 0, 0).grid(row=1, column=1)
        self.make_move_btn(frame_xyz, "Right (+X)", 0.04, 0, 0, 0, 0, 0).grid(row=1, column=2)
        self.make_move_btn(frame_xyz, "Up (-Y)", 0, -0.04, 0, 0, 0, 0).grid(row=0, column=3, padx=20)
        self.make_move_btn(frame_xyz, "Down (+Y)", 0, 0.04, 0, 0, 0, 0).grid(row=1, column=3, padx=20)

        ttk.Separator(frame_teleop, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # RPY Grid
        frame_rpy = tk.Frame(frame_teleop)
        frame_rpy.pack(pady=10)
        self.make_move_btn(frame_rpy, "Pitch Down (+Rx)", 0, 0, 0, 0.09, 0, 0).grid(row=0, column=1)
        self.make_move_btn(frame_rpy, "Yaw Right (-Ry)", 0, 0, 0, 0, -0.09, 0).grid(row=1, column=0)
        self.make_move_btn(frame_rpy, "Pitch Up (-Rx)", 0, 0, 0, -0.09, 0, 0).grid(row=1, column=1)
        self.make_move_btn(frame_rpy, "Yaw Left (+Ry)", 0, 0, 0, 0, 0.09, 0).grid(row=1, column=2)
        self.make_move_btn(frame_rpy, "Roll CCW (-Rz)", 0, 0, 0, 0, 0, -0.09).grid(row=0, column=3, padx=20)
        self.make_move_btn(frame_rpy, "Roll CW (+Rz)", 0, 0, 0, 0, 0, 0.09).grid(row=1, column=3, padx=20)

    # --- 核心：连续发布循环 (10Hz) ---
    def publish_move_loop(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return
            
        # 只要不处于 Auto 模式，就持续发布 (有按键发布速度，没按键发布全 0)，保持连接活跃
        if not self.in_auto_mode:
            self.pub.publish(self.current_twist)
            
        self.root.after(100, self.publish_move_loop) 

    # --- Teleop Logic ---
    def make_move_btn(self, parent, text, x, y, z, ax, ay, az):
        btn = tk.Button(parent, text=text, width=12, height=2, bg="lightgray")
        
        # 安全闭包绑定，同时增加按键变黄的视觉反馈
        def on_press(event, _x=x, _y=y, _z=z, _ax=ax, _ay=ay, _az=az):
            btn.config(bg="yellow")
            self.start_move(_x, _y, _z, _ax, _ay, _az)
            
        def on_release(event):
            btn.config(bg="lightgray")
            self.stop_move()
            
        btn.bind('<ButtonPress-1>', on_press)
        btn.bind('<ButtonRelease-1>', on_release)
        return btn

    def start_move(self, x, y, z, ax, ay, az):
        if self.in_auto_mode:
            self.in_auto_mode = False
            self.lbl_status.config(text="Mode: MANUAL", fg="blue")
            try:
                self.srv_servo_toggle(False)
                self.srv_direct_enable(False)
            except: pass
            
        # 【关键修复】确保机械臂管理器进入手动接收模式
        try: 
            self.srv_manual_toggle(True)
        except: pass

        # 更新当前发送的 Twist 目标值
        self.current_twist.linear.x = float(x)
        self.current_twist.linear.y = float(y)
        self.current_twist.linear.z = float(z)
        self.current_twist.angular.x = float(ax)
        self.current_twist.angular.y = float(ay)
        self.current_twist.angular.z = float(az)

    def stop_move(self):
        # 按钮松开时，将数值全部重置为 0，循环会自动发布 0 从而刹车
        self.current_twist = Twist()

    def stop_all(self):
        self.stop_move()
        self.in_auto_mode = False
        self.lbl_status.config(text="Mode: MANUAL", fg="blue")
        try: 
            self.srv_manual_toggle(True)
            self.srv_servo_toggle(False)
            self.srv_direct_enable(False)
        except: pass
        rospy.loginfo(">> EMERGENCY STOP / MANUAL OVERRIDE")
        self.pub.publish(Twist()) # 强制立刻发一帧 0 刹车

    # --- Commands ---
    def apply_settings(self):
        c_cam = self.var_cam.get()
        c_algo = self.var_algo.get()
        c_depth = self.var_depth.get()

        old_cam = rospy.get_param("/vs_controller/primary_camera", "")
        if c_cam != old_cam:
            messagebox.showwarning("Hardware Changed", f"Camera changed to {c_cam.upper()}.\n\nThis requires reloading hardware drivers. Please CLOSE this UI, press Ctrl+C in your launch terminal, and restart roslaunch!")
        
        rospy.set_param("/vs_controller/primary_camera", c_cam)
        rospy.set_param("/vs_controller/algorithm_type", c_algo)
        rospy.set_param("/vs_controller/depth_strategy", c_depth)

        self.config_mgr.update_yaml("primary_camera", c_cam)
        self.config_mgr.update_yaml("algorithm_type", c_algo)
        self.config_mgr.update_yaml("depth_strategy", c_depth)
        self.config_mgr.update_launch_default("primary_camera", c_cam)
        self.config_mgr.update_launch_default("algo_type", c_algo)
        self.config_mgr.update_launch_default("depth_strategy", c_depth)

        try:
            resp = self.srv_reload_config()
            rospy.loginfo(f"Reload: {resp.message}")
            messagebox.showinfo("Success", "Parameters applied and algorithm reloaded successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to reload config: {e}")

    def cmd_save_target(self):
        try: 
            self.srv_save_target()
            rospy.loginfo(">> Target Saved")
        except Exception as e: rospy.logerr(e)

    def cmd_auto_servo(self):
        self.in_auto_mode = True
        self.lbl_status.config(text="Mode: AUTO SERVO", fg="green")
        try:
            self.srv_servo_toggle(True)
            self.srv_direct_enable(True)
            rospy.loginfo(">> Auto Servo Started")
        except Exception as e: rospy.logerr(e)

    def cmd_move_slot(self, idx):
        rospy.set_param("/vs_manager/current_slot_id", idx)
        try: self.srv_move_slot()
        except: pass

    def cmd_save_slot(self, idx):
        rospy.set_param("/vs_manager/current_slot_id", idx)
        try: 
            self.srv_save_slot()
            messagebox.showinfo("Saved", f"Current pose saved to Slot {idx}")
        except: pass

    def cmd_home(self):
        try: self.srv_home()
        except: pass

    def on_closing(self):
        self.stop_all()
        self.root.destroy()
        rospy.signal_shutdown("GUI closed")

if __name__ == "__main__":
    root = tk.Tk()
    app = VSDashboard(root)
    root.mainloop()