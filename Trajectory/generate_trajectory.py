####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
# generate_trajectory.py

import numpy as np
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig

def generate_trajectory():
    """
    Generate a closed-loop race-like trajectory with defined lane boundaries.
    """
    trajectory_one = TrajectoryGenerator.generateTrajectory(
        Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        [Translation2d(5, 5), Translation2d(10, 0)],
        Pose2d(10, -5, Rotation2d.fromDegrees(-90)),
        TrajectoryConfig(3.0, 3.0)
    )

    trajectory_two = TrajectoryGenerator.generateTrajectory(
        Pose2d(10, -5, Rotation2d.fromDegrees(-90)),
        [Translation2d(5, -10), Translation2d(0, -5)],
        Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        TrajectoryConfig(3.0, 3.0)
    )

    combined_trajectory = trajectory_one.states() + trajectory_two.states()
    path = [(state.pose.X(), state.pose.Y()) for state in combined_trajectory]

    outer_left_boundary = []
    middle_left_boundary = []
    inner_left_boundary = []
    right_boundary = []
    lane_offset = 1.0  # Distance from the main path

    for i in range(len(path)):
        current = np.array(path[i])
        next_point = np.array(path[(i + 1) % len(path)])  # Next point (loop around)
        previous_point = np.array(path[i - 1])  # Previous point

        # Calculate direction vectors
        forward_direction = next_point - current
        backward_direction = current - previous_point

        # Average the direction vectors to smooth the transition
        avg_direction = (forward_direction + backward_direction) / 2.0
        avg_direction = avg_direction / np.linalg.norm(avg_direction)

        # Calculate perpendicular vector
        perpendicular_vector = np.array([-avg_direction[1], avg_direction[0]])

        # Calculate lane boundaries
        outer_left_boundary.append(tuple(current + lane_offset * 3 * perpendicular_vector))
        middle_left_boundary.append(tuple(current + lane_offset * 2 * perpendicular_vector))
        inner_left_boundary.append(tuple(current + lane_offset * perpendicular_vector))
        right_boundary.append(tuple(current - lane_offset * perpendicular_vector))

    return path, outer_left_boundary, middle_left_boundary, inner_left_boundary, right_boundary
