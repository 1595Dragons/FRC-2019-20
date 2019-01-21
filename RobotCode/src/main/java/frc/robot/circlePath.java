package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class circlePath {

    private RobotMap robot;

    private final String k_path_name = "Return to Start";

    public circlePath(RobotMap robot) {
        this.robot = robot;
    }

    public void init() {
        Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
        Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
    }


    public void periodic() {

    }
}