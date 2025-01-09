import os
import matplotlib.pyplot as plt
import pyqtgraph as pg
import numpy as np
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.optimize import minimize
import sys

from Trajectory.generate_trajectory import generate_trajectory
from Lateral_control.lateral_control_proportional import lateral_control
from Model.vehicle_model import VehicleModel
from Autopilot.autopilot import autopilot_step  

timeUpdate = 100 * 10**-3  # s


class StarterCode(QWidget):
    def __init__(self):
        super().__init__()
        self.time = 0
        self.setWindowTitle("Vehicle Control")

        # Create the main layout
        layout = QVBoxLayout(self)
        self.setLayout(layout)

        # Create the user interface widget
        self.ui = Interface()
        layout.addWidget(self.ui)

        # Create a timer for periodic updates
        self.timer = QTimer(self)

        # Connect the timer to the periodic update function
        self.timer.start(int(timeUpdate * 10**3))
        self.timer.timeout.connect(self.starterStep)
        self.timer.timeout.connect(self.ui.update_plot)

    def starterStep(self):
        """
        This function is called on each timer tick to perform periodic updates.
        """
        self.ui.time += timeUpdate

        # Autopilot: Use lateral control to calculate steering angle
        if self.ui.autopilot_is_pushed:
            steering_angle = autopilot_step(
                self.ui.pos_x_temp, self.ui.pos_y_temp, self.ui.path, self.ui.theta_temp, timeUpdate
            )
            self.ui.steering_temp.append(steering_angle)

        # Manual mode: Update direction based on key presses
        if self.ui.manual_mode:
            self.ui.update_manual_control()

        # Update vehicle position based on speed and direction
        speed = 1  # Fixed speed for testing
        direction = self.ui.theta_temp[-1]
        self.ui.velocity_temp.append(speed)
        self.ui.pos_x_temp.append(self.ui.pos_x_temp[-1] + speed * np.cos(direction) * timeUpdate)
        self.ui.pos_y_temp.append(self.ui.pos_y_temp[-1] + speed * np.sin(direction) * timeUpdate)


class Interface(QWidget):
    def __init__(self):
        super().__init__()

        # Generate the trajectory using wpimath
        self.path, self.left_border, self.left_border1, self.left_border2, self.right_border = generate_trajectory()

        # Initialize the VehicleModel object
        self.vehicle_model = VehicleModel()  # Model to manage vehicle's position and orientation

        # Temporary storage for plotting
        self.time = 0
        self.pos_x_temp = [0]  # Initial position of the vehicle
        self.pos_y_temp = [0]
        self.theta_temp = [0]  # Initial orientation
        self.steering_temp = []
        self.velocity_temp = []

        # Control flags
        self.manual_mode = False
        self.autopilot_is_pushed = True
        self.manual_steering_angle = 0  # Steering angle in manual mode

        # Create GUI components
        self.plot_simulator = pg.PlotWidget()
        self.plot_speed = pg.PlotWidget()
        self.plot_steering = pg.PlotWidget()
        self.error_message_box = QLabel("NO ERROR")
        self.error_message_box.setStyleSheet("background-color: red")
        self.autopilot = QPushButton("Autopilot")
        self.manual_mode_button = QPushButton("Manual Mode")

        self.autopilot.setChecked(True)
        self.autopilot.setStyleSheet("background-color: green")
        self.manual_mode_button.setStyleSheet("background-color: lightgray")

        # Layout
        layout = QGridLayout(self)
        self.setLayout(layout)

        # Add widgets to layout
        layout.addWidget(self.plot_speed, 0, 0, 1, 2)  # Speed graph at the top left
        layout.addWidget(self.plot_steering, 1, 0, 1, 2)  # Steering graph below the speed graph
        layout.addWidget(self.plot_simulator, 0, 2, 2, 4)  # Simulator occupying two rows and four columns
        layout.addWidget(self.error_message_box, 3, 0, 1, 2)  # Error message box spanning the entire width
        layout.addWidget(self.autopilot, 3, 2, 1, 2)  # Autopilot button in the center
        layout.addWidget(self.manual_mode_button, 3, 4, 1, 2)  # Manual mode button next to the autopilot button

        # Configure the plots
        self.plot_speed.setTitle("Vehicle Speed")
        self.plot_speed.setLabel("bottom", "Time (s)")
        self.plot_speed.setLabel("left", "Speed (m/s)")

        self.plot_steering.setTitle("Steering Angle")
        self.plot_steering.setLabel("bottom", "Time (s)")
        self.plot_steering.setLabel("left", "Angle (rad)")

        # Connect buttons
        self.autopilot.clicked.connect(self.toggle_piloting)
        self.manual_mode_button.clicked.connect(self.toggle_manual_mode)

    def sync_autopilot_state(self):
        """
        Synchronize states when switching to autopilot mode.
        """
        if not self.steering_temp:
            self.steering_temp.append(0)
        if not self.theta_temp:
            self.theta_temp.append(0)
        if not self.pos_x_temp or not self.pos_y_temp:
            self.pos_x_temp.append(0)
            self.pos_y_temp.append(0)

    def reset_manual_state(self):
        """
        Reset any specific states when switching to manual mode.
        """
        self.manual_steering_angle = 0

    def toggle_piloting(self):
        """Toggles autopilot."""
        self.autopilot_is_pushed = not self.autopilot_is_pushed
        if self.autopilot_is_pushed:
            self.autopilot.setStyleSheet("background-color: green;")
            self.manual_mode = False
            self.manual_mode_button.setStyleSheet("background-color: lightgray")
            self.sync_autopilot_state()  # Synchronize the state
        else:
            self.autopilot.setStyleSheet("background-color: lightgray;")
            self.manual_mode = True
            self.manual_mode_button.setStyleSheet("background-color: green")

    def toggle_manual_mode(self):
        """Toggles manual mode."""
        self.manual_mode = not self.manual_mode
        if self.manual_mode:
            self.manual_mode_button.setStyleSheet("background-color: green;")
            self.autopilot_is_pushed = False
            self.autopilot.setStyleSheet("background-color: lightgray")
            self.reset_manual_state()
        else:
            self.manual_mode_button.setStyleSheet("background-color: lightgray;")
            self.autopilot_is_pushed = True
            self.autopilot.setStyleSheet("background-color: green;")

    def plot_path(self):
        """Plots the trajectory on the simulator."""
        path_x, path_y = zip(*self.path)
        left_x, left_y = zip(*self.left_border)
        left_x1, left_y1 = zip(*self.left_border1)
        left_x2, left_y2 = zip(*self.left_border2)
        right_x, right_y = zip(*self.right_border)
        self.plot_simulator.plot(
            path_x, path_y,
            pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine),  # Blue line for trajectory
            name="Path"
        )
        self.plot_simulator.plot(
            left_x, left_y,
            pen=pg.mkPen('r', width=2),  # red line for left border
            name="Left_border"
        )
        self.plot_simulator.plot(
            left_x1, left_y1,
            pen=pg.mkPen('w', width=2),  # red line for left border1
            name="Left_border1"
        )
        self.plot_simulator.plot(
            left_x2, left_y2,
            pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine),  # red line for left border2
            name="Left_border2"
        )
        self.plot_simulator.plot(
            right_x, right_y,
            pen=pg.mkPen('r', width=2),  # red line for right border
            name="Right_border"
        )

    def update_manual_control(self):
        """
        Updates the vehicle orientation based on manual steering angle.
        """
        self.theta_temp[-1] += self.manual_steering_angle * timeUpdate

    def update_plot(self):
        """
        Updates the plot with the vehicle's position and trajectory.
        """
        if not self.theta_temp:
            return

        # Update vehicle position
        pos_x, pos_y = self.pos_x_temp[-1], self.pos_y_temp[-1]
        theta = self.theta_temp[-1]

        rectangle_x = np.array([-1, 1, 1, -1, -1])
        rectangle_y = np.array([-0.5, -0.5, 0.5, 0.5, -0.5])

        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        rotated_rectangle = np.dot(rotation_matrix, np.vstack((rectangle_x, rectangle_y)))
        rotated_rectangle[0, :] += pos_x
        rotated_rectangle[1, :] += pos_y

        self.plot_simulator.clear()
        self.plot_path()  # Draw trajectory
        self.plot_simulator.plot(rotated_rectangle[0, :], rotated_rectangle[1, :])
        self.plot_simulator.plot(
            [pos_x],
            [pos_y],
            pen=None,
            symbol="o",
            symbolPen=None,
            symbolSize=10,
            symbolBrush="r",
            name="Vehicle"
        )

        # Update speed and steering plots
        self.plot_speed.clear()
        self.plot_speed.plot(
            [t for t in range(len(self.velocity_temp))],
            self.velocity_temp,
            pen=pg.mkPen('r', width=2)
        )
        self.plot_steering.clear()
        self.plot_steering.plot(
            [t for t in range(len(self.steering_temp))],
            self.steering_temp,
            pen=pg.mkPen('g', width=2)
        )

    def keyPressEvent(self, event):
        """
        Handle key presses for manual control.
        """
        if self.manual_mode:
            if event.key() == Qt.Key_Left:
                self.manual_steering_angle = 1  # Turn right (inverted)
            elif event.key() == Qt.Key_Right:
                self.manual_steering_angle = -1  # Turn left (inverted)

    def keyReleaseEvent(self, event):
        """
        Handle key releases for manual control.
        """
        if self.manual_mode and event.key() in [Qt.Key_Left, Qt.Key_Right]:
            self.manual_steering_angle = 0  # Stop steering


