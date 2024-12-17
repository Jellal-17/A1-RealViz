#!/home/unitree/parkour/parkour_v/bin/python

import rospy
from sensor_msgs.msg import JointState
import sys
import tkinter as tk
from tkinter import messagebox
from unitree_legged_msgs.msg import LegsCmd
from unitree_legged_msgs.msg import LowState


# Define safety limits for each joint (in radians)
SAFETY_LIMITS = {
    "FR_hip_joint": {"min": -0.802, "max": 0.802},
    "FR_thigh_joint": {"min": -1.05, "max": 4.19},
    "FR_calf_joint": {"min": -2.7, "max": -0.916},
    "FL_hip_joint": {"min": -0.802, "max": 0.802},
    "FL_thigh_joint": {"min": -1.05, "max": 4.19},
    "FL_calf_joint": {"min": -2.7, "max": -0.916},
    "RR_hip_joint": {"min": -0.802, "max": 0.802},
    "RR_thigh_joint": {"min": -1.05, "max": 4.19},
    "RR_calf_joint": {"min": -2.7, "max": -0.916},
    "RL_hip_joint": {"min": -0.802, "max": 0.802},
    "RL_thigh_joint": {"min": -1.05, "max": 4.19},
    "RL_calf_joint": {"min": -2.7, "max": -0.916},
}

class JointStateGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Joint State Viewer")
        rospy.init_node('joint_state_gui', anonymous=True)

        # Initialize publishers and subscribers
        self.joint_pub = rospy.Publisher('/desired_joint_positions', JointState, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.low_state_subscriber = rospy.Subscriber("/low_state" ,LowState,self.low_state_callback,queue_size= 1,)

        self.low_state = None

        # GUI Elements
        self.joint_sliders = {}
        self.joint_labels = {}
        self.joint_positions = {}
        self.create_joint_controls()
        self.initialized = False

        self.desired_positions = {joint: 0.0 for joint in SAFETY_LIMITS.keys()}

        duration = 500
        rospy.Timer(rospy.Duration(1/ duration), self.timer_callback) #500Hz
    #     self.user_interacting = False

    # def on_slider_press(self):
    #     self.user_interacting = True
    
    # def on_slider_release(self):
    #     self.user_interacting = False
    

    def create_joint_controls(self):
        joints = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]

        for idx, joint in enumerate(joints):
            # Label for the joint
            label = tk.Label(self.root, text=joint)
            label.grid(row=idx, column=0, padx=10, pady=5)
            self.joint_labels[joint] = label

            # Slider for the joint with safety limits
            slider = tk.Scale(
                self.root,
                from_=SAFETY_LIMITS[joint]["min"],
                to=SAFETY_LIMITS[joint]["max"],
                orient=tk.HORIZONTAL,
                resolution=0.01,
                length=300
            )
            slider.set(0.0)  # Initial value; will be updated by callback
            slider.grid(row=idx, column=1, padx=10, pady=5)
            self.joint_sliders[joint] = slider
            # slider.bind("<ButtonPress>", self.on_slider_press)
            # slider.bind("<ButtonRelease>", self.on_slider_release)

        # Update button to send commands
        self.update_button = tk.Button(
            self.root,
            text="Update Joint Positions",
            command=self.publish_joint_positions
        )
        self.update_button.grid(row=len(joints), column=0, columnspan=2, pady=20)

    def joint_state_callback(self, msg):
        if not hasattr(self, 'initialized') or not self.initialized:

            for name, position in zip(msg.name, msg.position):
                if name in self.joint_sliders: 
                    self.joint_sliders[name].set(position)
                    self.joint_positions[name] = position
            self.initialized = True
            rospy.loginfo("JointStateGUI: Initialization complete")
            return

        # if not self.user_interacting:
            # for name, position in zip(msg.name, msg.position):
            #     if name in self.joint_sliders: 
            #         self.joint_sliders[name].set(position)
            #         self.joint_positions[name] = position
            # pass

        for name, position in zip(msg.name, msg.position):
            if name in self.joint_sliders: 
                # self.joint_sliders[name].set(position)
                self.joint_positions[name] = position

    def publish_joint_positions(self):
        if self.low_state is None: 
            rospy.logwarn("LowState not received yet")
            return
        # Collect current slider positions and publish as JointState
        
        warnings = []
        for joint in self.joint_sliders.keys():
            pos = self.joint_sliders[joint].get()
            min_limit = SAFETY_LIMITS[joint]["min"]
            max_limit = SAFETY_LIMITS[joint]["max"]
            clamped_pos = max(min_limit, min(pos, max_limit))
            self.desired_positions[joint] = clamped_pos
            if pos != clamped_pos:
                warnings.append(f"{joint} clamped to {clamped_pos:.2f} rad")
        
        if warnings:
            warning_message = "\n".join(warnings)
            messagebox.showwarning("Safety Limit Clamped", warning_message)
    
    def timer_callback(self, event):

        if self.low_state is None:
            return

        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = list(self.desired_positions.keys())

        joint_msg.position = [self.desired_positions[joint] for joint in joint_msg.name]
        self.joint_pub.publish(joint_msg)

    def low_state_callback(self, msg):
        self.low_state = msg        


def main():
    root = tk.Tk()
    gui = JointStateGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
