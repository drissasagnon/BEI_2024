####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
# IHM.py

import os
import time
import winsound
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
        speed = 1  # Initial speed
        self.ui.time += timeUpdate
        # Initialize steering_angle with a default value
        steering_angle = 0

        # Failure mode activated (ECU failure)
        if self.ui.failure_mode:
            speed, steering_angle, stop_message = safety_mecanism(
            True,
            self.ui.path,
            self.ui.pos_x_temp,
            self.ui.pos_y_temp,
            self.ui.theta_temp,
            self.ui.velocity_temp,
            self.ui.time,
            timeUpdate
            )
            self.ui.steering_temp.append(steering_angle)
            # Display stop message if vehicle has halted
            if stop_message:
                self.ui.error_message_box.setText(stop_message)
                

        # Autopilot mode activated (only if failure mode is deactivated)
        elif self.ui.autopilot_is_pushed:
            steering_angle = autopilot_step(
                self.ui.pos_x_temp, self.ui.pos_y_temp, self.ui.path, self.ui.theta_temp, timeUpdate, speed
            )
            self.ui.steering_temp.append(steering_angle)

        # Manual mode activated
        if self.ui.manual_mode:
            steering_angle = self.ui.manual_steering_angle

        # Update the position using VehicleModel
        self.ui.vehicle_model.update_position(steering_angle, speed, timeUpdate)

        # Update variables used for display
        pos_x, pos_y, theta = self.ui.vehicle_model.get_position()
        self.ui.pos_x_temp.append(pos_x)
        self.ui.pos_y_temp.append(pos_y)
        self.ui.theta_temp.append(theta)
        self.ui.velocity_temp.append(speed)
        self.ui.steering_temp.append(steering_angle)
        # Compute the error between the calculated and applied steering angle
        steering_angle_applied = self.ui.steering_temp[-1] if self.ui.steering_temp else 0
        steering_error = abs(steering_angle - steering_angle_applied)

        # 📝 Store the error in the list
        self.ui.steering_error_temp.append(steering_error)

class Interface(QWidget):
    def __init__(self):
        super().__init__()

        # Generate the trajectory using wpimath
        self.path, self.outer_left_boundary, self.middle_left_boundary, self.inner_left_boundary, self.right_boundary = generate_trajectory()


        # Initialize the VehicleModel object
        self.vehicle_model = VehicleModel()  # Model to manage vehicle's position and orientation

        # Temporary storage for plotting
        self.time = 0
        self.pos_x_temp = [0]  # Initial position of the vehicle
        self.pos_y_temp = [0]
        self.theta_temp = [0]  # Initial orientation
        self.steering_temp = []
        self.velocity_temp = []
        self.steering_error_temp = []  # ✅ Store steering error

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
        self.plot_error_steering = pg.PlotWidget()
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
        layout.addWidget(self.plot_error_steering, 2, 0, 1, 2)  # Steering erreur graph below the speed graph
        layout.addWidget(self.plot_simulator, 0, 2, 3, 4)  # Simulator occupying two rows and four columns
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

        self.plot_error_steering.setTitle("Steering erreur Angle")
        self.plot_error_steering.setLabel("bottom", "Time (s)")
        self.plot_error_steering.setLabel("left", "Angle (rad)")
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
        """Activate or deactivate safety mode (ECU failure)."""
        self.failure_mode = not self.failure_mode  # Toggle failure mode
         # 📂 Define log file path
        log_dir = os.path.join(os.getcwd(), "Logs")
        os.makedirs(log_dir, exist_ok=True)  # Ensure directory exists
        log_file_path = os.path.join(log_dir, "failure_log.txt")
        sound_path = os.path.join(os.getcwd(), "Safety_mecanism", "Alarme.wav")
        if self.failure_mode:
            # 🔊 Play alarm sound
            winsound.PlaySound(sound_path, winsound.SND_FILENAME | winsound.SND_ASYNC)


            # Disable autopilot
            self.autopilot_is_pushed = False
            self.autopilot.setStyleSheet("background-color: lightgray;")
            
            # Clear previous error messages
            self.error_message_box.setText(
                "⚠️ Failure detected! Activating safe mode.\n"
                "🚨 ECU failure reported to lateral controller.\n"
                "⚠️ Hazard lights and warnings activated.\n"
                "📍 New Parking Trajectory Calculated."
            )
            self.error_message_box.setStyleSheet("background-color: yellow; color: black;")
            self.blink_timer.start(500) 
            
            self.time = 0
            # 📝 Log failure event
            with open(log_file_path, "a", encoding="utf-8") as log_file:
                log_file.write(f"\n=== FAILURE MODE ACTIVATED ===\n")
                log_file.write(f"🚨 ECU failure detected at: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file.write(f"📍 Vehicle Position: ({self.pos_x_temp[-1]:.2f}, {self.pos_y_temp[-1]:.2f})\n")
                log_file.write("🚦 Switching to safety trajectory...\n")
        else:
            # Reactivate autopilot
            self.autopilot_is_pushed = True
            self.autopilot.setStyleSheet("background-color: green;")
            # Stop the blinking effect
            self.blink_timer.stop()
            winsound.PlaySound(None, winsound.SND_PURGE)
            # Reset error messages
            self.error_message_box.setText("Autopilot reactivated.")
            self.error_message_box.setStyleSheet("background-color: green; color: white;")
            QTimer.singleShot(3000, self.reset_error_message)  # Reset after 3 seconds
            # 📝 Log deactivation and final position
            with open(log_file_path, "a", encoding="utf-8") as log_file:
                log_file.write(f"\n✅ FAILURE MODE DEACTIVATED\n")
                log_file.write(f"🕒 DEACTIVATION ECU failure detected at: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file.write("============================================\n")

    
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

        # Récupérer la position actuelle et l'orientation du véhicule
        pos_x, pos_y, theta = self.vehicle_model.get_position()

        # Obtenir le rectangle représentant le véhicule
        rotated_rectangle = self.vehicle_model.get_rectangle()

        self.plot_simulator.clear()
        self.plot_path()  # Dessiner la trajectoire
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

        # Mise à jour des graphes de vitesse et d'angle de direction
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
        self.plot_error_steering.clear()
        self.plot_error_steering.plot(
            [t for t in range(len(self.steering_error_temp))],
            self.steering_error_temp,
            pen=pg.mkPen('g', width=2)
        )

    def plot_path(self):
        """Plots the trajectory on the simulator."""
        path_x, path_y = zip(*self.path)
        outer_left_x, outer_left_y = zip(*self.outer_left_boundary)
        middle_left_x, middle_left_y = zip(*self.middle_left_boundary)
        inner_left_x, inner_left_y = zip(*self.inner_left_boundary)
        right_x, right_y = zip(*self.right_boundary)

        self.plot_simulator.plot(
            path_x, path_y,
            pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine),  # Blue line for trajectory
            name="Path"
        )
        self.plot_simulator.plot(
            outer_left_x, outer_left_y,
            pen=pg.mkPen('r', width=2),  # Red line for outer left boundary
            name="Outer Left Boundary"
        )
        self.plot_simulator.plot(
            inner_left_x, inner_left_y,
            pen=pg.mkPen('w', width=2),  # White line for middle left boundary
            name="Middle Left Boundary"
        )
        self.plot_simulator.plot(
            middle_left_x, middle_left_y,
            pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine),  # Blue dashed line for inner left boundary
            name="Inner Left Boundary"
        )
        self.plot_simulator.plot(
            right_x, right_y,
            pen=pg.mkPen('r', width=2),  # Red line for right boundary
            name="Right Boundary"
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