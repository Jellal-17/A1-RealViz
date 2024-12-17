#!/home/unitree/parkour/parkour_v/bin/python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Lock
import numpy as np

class RealTimePlotter:
    def __init__(self):
        rospy.init_node('real_time_plotter', anonymous=True)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/foot_forces', Float32MultiArray, self.foot_forces_callback)

        self.lock = Lock()
        self.time_data = []
        self.joint_positions = [[] for _ in range(12)]
        self.foot_forces = [[] for _ in range(4)]

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 10))

        # Create line objects for each joint
        self.lines_joint = [self.ax1.plot([], [], label=f'Joint {i}')[0] for i in range(12)]

        # Create line objects for foot forces
        self.lines_foot = [self.ax2.plot([], [], label=f'Foot {i}')[0] for i in range(4)]

        # Set up plot parameters
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Joint Position (rad)')
        self.ax1.legend(loc='upper right')

        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Foot Force')
        self.ax2.legend(loc='upper right')

        # Animation function
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

    def joint_states_callback(self, msg):
        with self.lock:
            current_time = rospy.get_time()
            self.time_data.append(current_time)
            for i in range(len(msg.position)):
                self.joint_positions[i].append(msg.position[i])

            # Keep only the latest N data points
            N = 200
            self.time_data = self.time_data[-N:]
            for i in range(12):
                self.joint_positions[i] = self.joint_positions[i][-N:]

    def foot_forces_callback(self, msg):
        with self.lock:
            for i in range(len(msg.data)):
                self.foot_forces[i].append(msg.data[i])

            # Keep only the latest N data points
            N = 200
            for i in range(4):
                self.foot_forces[i] = self.foot_forces[i][-N:]

    def update_plot(self, frame):
        with self.lock:
            if not self.time_data:
                return

            # Convert lists to numpy arrays for plotting
            time_array = np.array(self.time_data) - self.time_data[0]  # Start time at zero

            # Update joint positions plot
            for i, line in enumerate(self.lines_joint):
                line.set_data(time_array, self.joint_positions[i])

            self.ax1.relim()
            self.ax1.autoscale_view()

            # Update foot forces plot
            for i, line in enumerate(self.lines_foot):
                line.set_data(time_array, self.foot_forces[i])

            self.ax2.relim()
            self.ax2.autoscale_view()

        return self.lines_joint + self.lines_foot

    def run(self):
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        plotter = RealTimePlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
