import os
import time
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from Trajectory.generate_trajectory import generate_trajectory
from Model.vehicle_model import VehicleModel
from Autopilot.autopilot import autopilot_step
from Safety_mecanism.Safety_mecanism import safety_mecanism

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
        if self.ui.failure_mode:
            # Ne rien faire si le mode de défaillance est activé
            return

        self.ui.time += timeUpdate
        speed = 1
        # Autopilot: Use lateral control to calculate steering angle
        if self.ui.autopilot_is_pushed:
            steering_angle = autopilot_step(
                self.ui.pos_x_temp, self.ui.pos_y_temp, self.ui.path, self.ui.theta_temp, timeUpdate, speed
            )
            self.ui.steering_temp.append(steering_angle)

        # Manual mode: Update direction based on key presses
        if self.ui.manual_mode:
            self.ui.update_manual_control()

        # Update vehicle position based on speed and direction
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
        self.failure_mode = False  # New flag for failure mode
        self.manual_steering_angle = 0  # Steering angle in manual mode

         # Other initializations...
        self.blink_timer = QTimer(self)  # Timer for blinking
        self.blink_state = False  # Track the blinking state
        self.blink_timer.timeout.connect(self.toggle_blink)  # Connect to blinking function

        # Create GUI components
        self.plot_simulator = pg.PlotWidget()
        self.plot_speed = pg.PlotWidget()
        self.plot_steering = pg.PlotWidget()
        self.error_message_box = QLabel("NO ERROR")
        self.error_message_box.setStyleSheet("background-color: red")
        self.autopilot = QPushButton("Autopilot")
        self.manual_mode_button = QPushButton("Manual Mode")
        self.defaillance_button = QPushButton("DEFAILLANCE ECU")
        self.autopilot.setChecked(True)
        self.defaillance_button.setStyleSheet("background-color: red")
        self.autopilot.setStyleSheet("background-color: green")
        self.manual_mode_button.setStyleSheet("background-color: lightgray")

        # Layout
        layout = QGridLayout(self)
        self.setLayout(layout)

        # Add widgets to layout
        layout.addWidget(self.plot_speed, 0, 0, 1, 2)  # Speed graph at the top left
        layout.addWidget(self.plot_steering, 1, 0, 1, 2)  # Steering graph below the speed graph
        layout.addWidget(self.plot_simulator, 0, 2, 2, 4)  # Simulator occupying two rows and four columns
        layout.addWidget(self.error_message_box, 3, 0, 2, 2)  # Error message box spanning the entire width
        layout.addWidget(self.autopilot, 3, 2, 1, 2)  # Autopilot button in the center
        layout.addWidget(self.manual_mode_button, 3, 4, 1, 2)  # Manual mode button next to the autopilot button
        layout.addWidget(self.defaillance_button, 4, 2, 1, 4)  # Defaillance button

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
        self.defaillance_button.clicked.connect(self.toggle_failure_mode)

    def toggle_piloting(self):
        """Toggles autopilot."""
        if self.failure_mode:
            return  # Prevent toggling if in failure mode

        self.autopilot_is_pushed = not self.autopilot_is_pushed
        if self.autopilot_is_pushed:
            self.autopilot.setStyleSheet("background-color: green;")
            self.manual_mode = False
            self.manual_mode_button.setStyleSheet("background-color: lightgray")
        else:
            self.autopilot.setStyleSheet("background-color: lightgray;")
            self.manual_mode = True
            self.manual_mode_button.setStyleSheet("background-color: green")

    def toggle_manual_mode(self):
        """Toggles manual mode."""
        if self.failure_mode:
            return  # Prevent toggling if in failure mode

        self.manual_mode = not self.manual_mode
        if self.manual_mode:
            self.manual_mode_button.setStyleSheet("background-color: green;")
            self.autopilot_is_pushed = False
            self.autopilot.setStyleSheet("background-color: lightgray")
        else:
            self.manual_mode_button.setStyleSheet("background-color: lightgray;")
            self.autopilot_is_pushed = True
            self.autopilot.setStyleSheet("background-color: green;")

    def toggle_failure_mode(self):
        """Activate or deactivate failure mode."""
        self.failure_mode = not self.failure_mode
        if self.failure_mode:
            # Function to update error_message_box
            def update_error_box(message):
                current_text = self.error_message_box.text()
                self.error_message_box.setText(current_text + "\n" + message)

            # Activate safety mechanism with callback
            self.error_message_box.setText("")
            self.error_message_box.setStyleSheet("background-color: yellow; color: black;")
            safety_mecanism(
                self.vehicle_model,
                True,
                self.path,
                (self.pos_x_temp[-1], self.pos_y_temp[-1]),
                self.velocity_temp[-1] if self.velocity_temp else 0,
                callback=update_error_box
            )
            self.blink_timer.start(500)  # Blink every 500 ms

        else:
            # Reset the error message box
            self.blink_timer.stop()
            self.blink_state = False
            self.error_message_box.setText("Autopilot reactivated.")
            self.error_message_box.setStyleSheet("background-color: green; color: white;")
            self.autopilot_is_pushed = True
            QTimer.singleShot(3000, self.reset_error_message)

    def toggle_blink(self):
        """Toggle the blinking effect for error_message_box."""
        if self.blink_state:
            self.error_message_box.setStyleSheet("background-color: yellow; color: black;")
        else:
            self.error_message_box.setStyleSheet("background-color: black; color: yellow;")
        self.blink_state = not self.blink_state
    def reset_error_message(self):
        """Resets the error message to 'No Error'."""
        self.error_message_box.setText("NO ERROR")
        self.error_message_box.setStyleSheet("background-color: red; color: white;")

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
            pen=pg.mkPen('r', width=2),  # Red line for left border
            name="Left_border"
        )
        self.plot_simulator.plot(
            left_x1, left_y1,
            pen=pg.mkPen('w', width=2),  # White line for left border1
            name="Left_border1"
        )
        self.plot_simulator.plot(
            left_x2, left_y2,
            pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine),  # Blue dashed line for left border2
            name="Left_border2"
        )
        self.plot_simulator.plot(
            right_x, right_y,
            pen=pg.mkPen('r', width=2),  # Red line for right border
            name="Right_border"
        )
    def keyPressEvent(self, event):
        """
        Handle key presses for manual control.
        """
        if self.manual_mode:
            if event.key() == Qt.Key_Left:
                self.manual_steering_angle = 1  # Turn left
            elif event.key() == Qt.Key_Right:
                self.manual_steering_angle = -1  # Turn right

    def keyReleaseEvent(self, event):
        """
        Handle key releases for manual control.
        """
        if self.manual_mode and event.key() in [Qt.Key_Left, Qt.Key_Right]:
            self.manual_steering_angle = 0  # Stop steering