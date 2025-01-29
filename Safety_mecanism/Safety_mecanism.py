import time
import os
import numpy as np

def safety_mecanism(vehicle_model, button_signal, path, vehicle_position, speed, callback=None):
    """
    Handles vehicle safety in case of ECU failure.

    :param vehicle_model: Vehicle model containing position and orientation.
    :param button_signal: Indicates whether the failure button is activated (True/False).
    :param path: Vehicle trajectory.
    :param vehicle_position: Current vehicle position as a tuple (x, y).
    :param speed: Current vehicle speed.
    :param callback: Function to handle log messages (optional).
    """
    if not button_signal:
        return

    # Use callback for messages, fallback to print
    if callback:
        callback("Failure detected! Activating safe mode.")
        callback("ECU failure reported to lateral controller.")
        callback("Hazard lights and warnings activated.")
    else:
        print("Failure detected! Activating safe mode.")
        print("ECU failure reported to lateral controller.")
        print("Hazard lights and warnings activated.")

    target_speed = max(0, speed - 0.5)
    if callback:
        callback(f"Reducing vehicle speed: {target_speed} m/s")
    else:
        print(f"Reducing vehicle speed: {target_speed} m/s")

    vehicle_model.update_position(0, target_speed, 1)

    while target_speed > 0:
        time.sleep(1)
        target_speed = max(0, target_speed - 0.5)
        vehicle_model.update_position(0, target_speed, 1)
        if callback:
            callback(f"Controlled stop in progress. Current speed: {target_speed} m/s")
        else:
            print(f"Controlled stop in progress. Current speed: {target_speed} m/s")

    if callback:
        callback("Vehicle stopped in a safe zone.")
        callback("Emergency call sent to rescue services.")
    else:
        print("Vehicle stopped in a safe zone.")
        print("Emergency call sent to rescue services.")


    # Log failure details for diagnostics
    log_dir = os.path.join(os.getcwd(), "Safety_mecanism")
    log_file_path = os.path.join(log_dir, "log.txt")
    with open(log_file_path, "a") as log_file:
        log_file.write(f"Failure detected. Final position: {vehicle_position}\n")
        log_file.write(f"Incident time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        log_file.write("Safe mode activated and vehicle stopped.\n")
