package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

// https://wpilib.screenstepslive.com/s/currentCS/m/84338/l/1021631-integrating-path-following-into-a-robot-program
public class circlePath {

    private RobotMap robot;

    private static final int k_ticks_per_rev = 4060;
    private static final double k_wheel_diameter = 0.1524d; // 6 inches to meters
    private static final double k_max_velocity = 2.794d; // 110 inches to meters

    private final String k_path_name = "Return to start";

    private Notifier m_follower_notifier;

    private AHRS gyro;

    private EncoderFollower m_left_follower;
    private EncoderFollower m_right_follower;

    public circlePath(RobotMap robot) {
        this.robot = robot;
    }

    public void init() {
        this.gyro = new AHRS(SPI.Port.kMXP);

        // TODO: Error with trajectories
        Trajectory left_trajectory = PathfinderFRC.getTrajectory(this.k_path_name + ".left");
        Trajectory right_trajectory = PathfinderFRC.getTrajectory(this.k_path_name + ".right");

        this.m_left_follower = new EncoderFollower(left_trajectory);
        this.m_right_follower = new EncoderFollower(right_trajectory);

        this.m_left_follower.configureEncoder(this.robot.leftDrive1.getSelectedSensorPosition(), this.k_ticks_per_rev,
                this.k_wheel_diameter);
        // You must tune the PID values on the following line!
        m_left_follower.configurePIDVA(0.02d, 4.0E-4d, 0.0d, 1 / this.k_max_velocity, 0);

        this.m_right_follower.configureEncoder(this.robot.rightDrive1.getSelectedSensorPosition(), this.k_ticks_per_rev,
                this.k_wheel_diameter);
        // You must tune the PID values on the following line!
        this.m_right_follower.configurePIDVA(0.02d, 4.0E-4d, 0.0d, 1 / this.k_max_velocity, 0);

        this.m_follower_notifier = new Notifier(this::followPath);
        this.m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);

    }

    public void periodic() {

    }

    private void followPath() {
        if (this.m_left_follower.isFinished() || this.m_right_follower.isFinished()) {
            this.m_follower_notifier.stop();
        } else {
            double left_speed = this.m_left_follower.calculate(this.robot.leftDrive1.getSelectedSensorPosition());
            double right_speed = this.m_right_follower.calculate(this.robot.leftDrive1.getSelectedSensorPosition());
            double heading = this.gyro.getYaw();
            double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
            this.robot.leftDrive1.set(ControlMode.PercentOutput, left_speed + turn);
            this.robot.rightDrive1.set(ControlMode.PercentOutput, right_speed - turn);
        }
    }
}