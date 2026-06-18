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

# --- 现代 UI 调色板 ---
BG_MAIN = "#F4F6F9"        # 整体浅灰背景
BG_CARD = "#FFFFFF"        # 卡片纯白背景
TEXT_MAIN = "#2C3E50"      # 深蓝灰主文本
TEXT_LIGHT = "#7F8C8D"     # 浅灰次文本
COLOR_PRIMARY = "#3498DB"  # 主题蓝
COLOR_SUCCESS = "#2ECC71"  # 成功绿
COLOR_DANGER = "#E74C3C"   # 危险红
COLOR_WARNING = "#F1C40F"  # 警告/激活黄
COLOR_HOVER = "#E0E6ED"    # 按钮默认底色

FONT_TITLE = ("Segoe UI", 16, "bold")
FONT_HEADING = ("Segoe UI", 12, "bold")
FONT_MAIN = ("Segoe UI", 10)
FONT_BTN = ("Segoe UI", 10, "bold")

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
        self.root.geometry("850x700")
        self.root.configure(bg=BG_MAIN) # 设置主背景色
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

        self.setup_styles()
        self.create_widgets()
        
        self.publish_move_loop()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # 全局 TTK 样式美化
        style.configure(".", font=FONT_MAIN, background=BG_CARD, foreground=TEXT_MAIN)
        style.configure("TLabelframe", background=BG_MAIN, borderwidth=0)
        style.configure("TLabelframe.Label", font=FONT_HEADING, foreground=TEXT_MAIN, background=BG_MAIN)
        
        style.configure("TCombobox", padding=5)
        style.configure("TEntry", padding=5)
        style.configure("TCheckbutton", background=BG_CARD, font=FONT_MAIN)
        
        # 按钮样式
        style.configure("Primary.TButton", font=FONT_BTN, padding=6, background=COLOR_PRIMARY, foreground="white")
        style.map("Primary.TButton", background=[("active", "#2980B9")])
        
        style.configure("Card.TFrame", background=BG_CARD)

    def create_widgets(self):
        # === 1. TOP: Status & Emergency ===
        frame_top = tk.Frame(self.root, bg=TEXT_MAIN, pady=15, padx=20)
        frame_top.pack(fill=tk.X)
        
        self.lbl_status = tk.Label(frame_top, text="● MODE: MANUAL", font=FONT_TITLE, bg=TEXT_MAIN, fg="#3498DB")
        self.lbl_status.pack(side=tk.LEFT)
        
        btn_stop = tk.Button(frame_top, text="EMERGENCY STOP (SPACE)", font=FONT_BTN, bg=COLOR_DANGER, fg="white", 
                             relief="flat", padx=15, pady=5, cursor="hand2", command=self.stop_all)
        btn_stop.pack(side=tk.RIGHT)
        self.root.bind('<space>', lambda e: self.stop_all())

        # === 2. SETTINGS PANEL (美化参数与 Precond 区域) ===
        frame_settings_wrapper = ttk.LabelFrame(self.root, text="System Configuration")
        frame_settings_wrapper.pack(fill=tk.X, padx=20, pady=(15, 10))
        
        # 卡片式白底容器
        frame_settings = ttk.Frame(frame_settings_wrapper, style="Card.TFrame")
        frame_settings.pack(fill=tk.X, padx=5, pady=5, ipadx=10, ipady=10)

        self.var_cam = tk.StringVar(value=rospy.get_param("/vs_controller/primary_camera", "basler"))
        self.var_algo = tk.StringVar(value=rospy.get_param("/vs_controller/algorithm_type", "defocused"))
        self.var_depth = tk.StringVar(value=rospy.get_param("/vs_controller/depth_strategy", "constant"))
        self.var_precond = tk.BooleanVar(value=rospy.get_param("/vs_controller/enable_precond", True))
        self.var_scale = tk.StringVar(value=str(rospy.get_param("/vs_controller/precond_scale", 0.1)))

        # Row 0: Basic Params
        ttk.Label(frame_settings, text="Camera:").grid(row=0, column=0, sticky=tk.E, padx=(10, 5), pady=10)
        ttk.Combobox(frame_settings, textvariable=self.var_cam, values=["l515", "basler"], state="readonly", width=12).grid(row=0, column=1, padx=5, pady=10)

        ttk.Label(frame_settings, text="Algorithm:").grid(row=0, column=2, sticky=tk.E, padx=(20, 5), pady=10)
        ttk.Combobox(frame_settings, textvariable=self.var_algo, values=["dvs", "defocused", "depth", "defocused_RAL"], state="readonly", width=15).grid(row=0, column=3, padx=5, pady=10)

        ttk.Label(frame_settings, text="Depth:").grid(row=0, column=4, sticky=tk.E, padx=(20, 5), pady=10)
        ttk.Combobox(frame_settings, textvariable=self.var_depth, values=["realtime", "constant", "desired_only"], state="readonly", width=12).grid(row=0, column=5, padx=5, pady=10)

        # 分割线
        ttk.Separator(frame_settings, orient=tk.HORIZONTAL).grid(row=1, column=0, columnspan=6, sticky="ew", padx=10, pady=5)

        # Row 2: Precondition Settings (优化排版)
        frame_precond = ttk.Frame(frame_settings, style="Card.TFrame")
        frame_precond.grid(row=2, column=0, columnspan=4, sticky=tk.W, padx=10, pady=5)
        
        ttk.Checkbutton(frame_precond, text="Enable Matrix Preconditioning", variable=self.var_precond).pack(side=tk.LEFT, padx=(0, 20))
        ttk.Label(frame_precond, text="Precond Scale:").pack(side=tk.LEFT, padx=(0, 5))
        self.entry_scale = ttk.Entry(frame_precond, textvariable=self.var_scale, width=10)
        self.entry_scale.pack(side=tk.LEFT)

        # Apply Button right-aligned
        btn_apply = ttk.Button(frame_settings, text="Apply & Reload", style="Primary.TButton", command=self.apply_settings)
        btn_apply.grid(row=2, column=4, columnspan=2, sticky=tk.E, padx=10, pady=5)

        # === 3. MAIN CONTROL & WAYPOINTS ===
        frame_mid = tk.Frame(self.root, bg=BG_MAIN)
        frame_mid.pack(fill=tk.BOTH, expand=True, padx=20, pady=(0, 20))

        # Left: Waypoints (卡片式)
        frame_wp_wrapper = ttk.LabelFrame(frame_mid, text="Waypoints & Tasks")
        frame_wp_wrapper.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        frame_wp = ttk.Frame(frame_wp_wrapper, style="Card.TFrame")
        frame_wp.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.create_flat_btn(frame_wp, "[M] SAVE TARGET IMAGE", COLOR_SUCCESS, "white", self.cmd_save_target).pack(fill=tk.X, padx=15, pady=(15, 5))
        self.create_flat_btn(frame_wp, "[V] START AUTO SERVO", COLOR_PRIMARY, "white", self.cmd_auto_servo).pack(fill=tk.X, padx=15, pady=5)

        ttk.Separator(frame_wp, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15, padx=15)

        for i in range(1, 5):
            f = ttk.Frame(frame_wp, style="Card.TFrame")
            f.pack(fill=tk.X, padx=15, pady=3)
            ttk.Button(f, text=f"Move Slot {i}", command=lambda idx=i: self.cmd_move_slot(idx)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
            ttk.Button(f, text=f"Save {i}", command=lambda idx=i: self.cmd_save_slot(idx)).pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=(2, 0))

        ttk.Separator(frame_wp, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15, padx=15)
        ttk.Button(frame_wp, text="[H] Home Robot", command=self.cmd_home).pack(fill=tk.X, padx=15, pady=(0, 15))

        # Right: Teleop
        frame_teleop_wrapper = ttk.LabelFrame(frame_mid, text="Teleop (Hold to Move)")
        frame_teleop_wrapper.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        frame_teleop = ttk.Frame(frame_teleop_wrapper, style="Card.TFrame")
        frame_teleop.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        lbl_tip = tk.Label(frame_teleop, text="Press and Hold buttons to move camera", bg=BG_CARD, fg=TEXT_LIGHT, font=FONT_MAIN)
        lbl_tip.pack(pady=10)

        # XYZ Grid
        frame_xyz = tk.Frame(frame_teleop, bg=BG_CARD)
        frame_xyz.pack(pady=5)
        self.make_move_btn(frame_xyz, "Forward\n(+Z)", 0, 0, 0.04, 0, 0, 0).grid(row=0, column=1, padx=3, pady=3)
        self.make_move_btn(frame_xyz, "Left\n(-X)", -0.04, 0, 0, 0, 0, 0).grid(row=1, column=0, padx=3, pady=3)
        self.make_move_btn(frame_xyz, "Backward\n(-Z)", 0, 0, -0.04, 0, 0, 0).grid(row=1, column=1, padx=3, pady=3)
        self.make_move_btn(frame_xyz, "Right\n(+X)", 0.04, 0, 0, 0, 0, 0).grid(row=1, column=2, padx=3, pady=3)
        self.make_move_btn(frame_xyz, "Up\n(-Y)", 0, -0.04, 0, 0, 0, 0).grid(row=0, column=3, padx=(25, 3), pady=3)
        self.make_move_btn(frame_xyz, "Down\n(+Y)", 0, 0.04, 0, 0, 0, 0).grid(row=1, column=3, padx=(25, 3), pady=3)

        ttk.Separator(frame_teleop, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15, padx=20)

        # RPY Grid
        frame_rpy = tk.Frame(frame_teleop, bg=BG_CARD)
        frame_rpy.pack(pady=5)
        self.make_move_btn(frame_rpy, "Pitch Dn\n(+Rx)", 0, 0, 0, 0.09, 0, 0).grid(row=0, column=1, padx=3, pady=3)
        self.make_move_btn(frame_rpy, "Yaw Rt\n(-Ry)", 0, 0, 0, 0, -0.09, 0).grid(row=1, column=0, padx=3, pady=3)
        self.make_move_btn(frame_rpy, "Pitch Up\n(-Rx)", 0, 0, 0, -0.09, 0, 0).grid(row=1, column=1, padx=3, pady=3)
        self.make_move_btn(frame_rpy, "Yaw Lf\n(+Ry)", 0, 0, 0, 0, 0.09, 0).grid(row=1, column=2, padx=3, pady=3)
        self.make_move_btn(frame_rpy, "Roll CCW\n(-Rz)", 0, 0, 0, 0, 0, -0.09).grid(row=0, column=3, padx=(25, 3), pady=3)
        self.make_move_btn(frame_rpy, "Roll CW\n(+Rz)", 0, 0, 0, 0, 0, 0.09).grid(row=1, column=3, padx=(25, 3), pady=3)

    # --- UI Helpers ---
    def create_flat_btn(self, parent, text, bg_color, fg_color, command):
        btn = tk.Button(parent, text=text, bg=bg_color, fg=fg_color, font=FONT_BTN, 
                        relief="flat", borderwidth=0, cursor="hand2", command=command, pady=8)
        # 悬停变色效果
        btn.bind("<Enter>", lambda e: btn.config(bg=self.adjust_color_lightness(bg_color, 1.1)))
        btn.bind("<Leave>", lambda e: btn.config(bg=bg_color))
        return btn

    def adjust_color_lightness(self, hex_color, factor):
        """简单计算悬停颜色的高亮辅助函数"""
        h = hex_color.lstrip('#')
        rgb = tuple(int(h[i:i+2], 16) for i in (0, 2, 4))
        new_rgb = tuple(min(int(c * factor), 255) for c in rgb)
        return '#%02x%02x%02x' % new_rgb

    def make_move_btn(self, parent, text, x, y, z, ax, ay, az):
        btn = tk.Button(parent, text=text, width=10, height=2, bg=COLOR_HOVER, fg=TEXT_MAIN,
                        relief="flat", font=("Segoe UI", 9, "bold"), cursor="hand2")
        
        def on_press(event, _x=x, _y=y, _z=z, _ax=ax, _ay=ay, _az=az):
            btn.config(bg=COLOR_WARNING, fg="black")
            self.start_move(_x, _y, _z, _ax, _ay, _az)
            
        def on_release(event):
            btn.config(bg=COLOR_HOVER, fg=TEXT_MAIN)
            self.stop_move()
            
        btn.bind('<ButtonPress-1>', on_press)
        btn.bind('<ButtonRelease-1>', on_release)
        return btn

    # --- 核心机制 (保持不变) ---
    def publish_move_loop(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return
        if not self.in_auto_mode:
            self.pub.publish(self.current_twist)
        self.root.after(100, self.publish_move_loop) 

    def start_move(self, x, y, z, ax, ay, az):
        if self.in_auto_mode:
            self.in_auto_mode = False
            self.lbl_status.config(text="● MODE: MANUAL", fg=COLOR_PRIMARY)
            try:
                self.srv_servo_toggle(False)
                self.srv_direct_enable(False)
            except: pass
        try: self.srv_manual_toggle(True)
        except: pass

        self.current_twist.linear.x = float(x)
        self.current_twist.linear.y = float(y)
        self.current_twist.linear.z = float(z)
        self.current_twist.angular.x = float(ax)
        self.current_twist.angular.y = float(ay)
        self.current_twist.angular.z = float(az)

    def stop_move(self):
        self.current_twist = Twist()

    def stop_all(self):
        self.stop_move()
        self.in_auto_mode = False
        self.lbl_status.config(text="● MODE: MANUAL", fg=COLOR_PRIMARY)
        try: 
            self.srv_manual_toggle(True)
            self.srv_servo_toggle(False)
            self.srv_direct_enable(False)
        except: pass
        rospy.loginfo(">> EMERGENCY STOP / MANUAL OVERRIDE")
        self.pub.publish(Twist())

    # --- Commands (保持不变) ---
    def apply_settings(self):
        c_cam = self.var_cam.get()
        c_algo = self.var_algo.get()
        c_depth = self.var_depth.get()
        c_precond = self.var_precond.get()
        try: c_scale = float(self.var_scale.get())
        except ValueError:
            messagebox.showerror("Error", "Precond Scale 必须是合法的浮点数！")
            return

        old_cam = rospy.get_param("/vs_controller/primary_camera", "")
        if c_cam != old_cam:
            messagebox.showwarning("Hardware Changed", f"Camera changed to {c_cam.upper()}.\n\nThis requires reloading hardware drivers. Please CLOSE this UI, press Ctrl+C in your launch terminal, and restart roslaunch!")
        
        rospy.set_param("/vs_controller/primary_camera", c_cam)
        rospy.set_param("/vs_controller/algorithm_type", c_algo)
        rospy.set_param("/vs_controller/depth_strategy", c_depth)
        rospy.set_param("/vs_controller/enable_precond", c_precond)
        rospy.set_param("/vs_controller/precond_scale", c_scale)

        self.config_mgr.update_yaml("primary_camera", c_cam)
        self.config_mgr.update_yaml("algorithm_type", c_algo)
        self.config_mgr.update_yaml("depth_strategy", c_depth)
        self.config_mgr.update_yaml("enable_precond", c_precond)
        self.config_mgr.update_yaml("precond_scale", c_scale)
        
        self.config_mgr.update_launch_default("primary_camera", c_cam)
        self.config_mgr.update_launch_default("algo_type", c_algo)
        self.config_mgr.update_launch_default("depth_strategy", c_depth)
        try:
            resp = self.srv_reload_config()
            messagebox.showinfo("Success", "Parameters applied and algorithm reloaded successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to reload config: {e}")

    def cmd_save_target(self):
        try: self.srv_save_target()
        except Exception as e: rospy.logerr(e)

    def cmd_auto_servo(self):
        self.in_auto_mode = True
        self.lbl_status.config(text="● MODE: AUTO SERVO", fg=COLOR_SUCCESS)
        try:
            self.srv_servo_toggle(True)
            self.srv_direct_enable(True)
        except Exception as e: rospy.logerr(e)

    def cmd_move_slot(self, idx):
        rospy.set_param("/vs_manager/current_slot_id", idx)
        try: self.srv_move_slot()
        except: pass

    def cmd_save_slot(self, idx):
        rospy.set_param("/vs_manager/current_slot_id", idx)
        try: self.srv_save_slot()
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