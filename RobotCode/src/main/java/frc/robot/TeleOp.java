package frc.robot;

import java.util.ConcurrentModificationException;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is where we put the code that we want to run during teleop.
 * 
 * In the event that either <code>init()</code> or <code>periodic()</code> throw
 * an error, this can be caught on the Robot.java end witha simple
 * <code>try{}catch{}</code> block.
 */
public class TeleOp {

    private RobotMap robot = new RobotMap();

    // For using a PID, centerX is our error
    private double kP, kI;

    private final double initialKP = 0.015d, initialKI = 0.003d;

    /**
     * This is code that we only want to run <b>once</b>.
     */
    public void init() {
        SmartDashboard.putNumber("kP", this.initialKP);
        SmartDashboard.putNumber("kI", this.initialKI);
    }

    /**
     * This is for code that we want to run <b>repeatidly</b>.
     */
    public void periodic(Vision vision) {

        // Calculate drive code, with turbo button
        double forward = robot.gamepad1.getStickButton(Hand.kLeft) ? robot.gamepad1.getY(Hand.kLeft)
                : robot.gamepad1.getY(Hand.kLeft) / 2,
                turn = robot.gamepad1.getStickButton(Hand.kLeft) ? robot.gamepad1.getX(Hand.kRight)
                        : robot.gamepad1.getX(Hand.kRight) / 2;

        // Basic west coast drive code
        if (Math.abs(forward) > 0.05d || Math.abs(turn) > 0.05d) {
            this.robot.leftDrive1.set(ControlMode.PercentOutput, forward - turn);
            this.robot.rightDrive1.set(ControlMode.PercentOutput, forward + turn);
        } else {
            this.robot.leftDrive1.set(ControlMode.PercentOutput, 0);
            this.robot.rightDrive1.set(ControlMode.PercentOutput, 0);
        }

        SmartDashboard.putNumber("Left drive 1 power", this.robot.leftDrive1.getMotorOutputPercent());
        SmartDashboard.putNumber("Right drive 1 power", this.robot.rightDrive1.getMotorOutputPercent());

        // Try to get the center of the line for tracking
        try {
            double centerX = vision.findCenterX();
            SmartDashboard.putNumber("Center X", centerX);

            if (robot.gamepad1.getAButtonPressed()) {
                SmartDashboard.putBoolean("Vision tracking", !SmartDashboard.getBoolean("Vision tracking", false));
            }

            if (SmartDashboard.getBoolean("Vision tracking", false) && centerX != 0) {

                // Get the PID stuff
                this.kP = SmartDashboard.getNumber("kP", this.initialKP);
                this.kI = SmartDashboard.getNumber("kI", this.initialKI);

                if (Math.abs(centerX) > 10) {
                    vision.trackTarget(centerX / 10, this.robot.leftDrive1, this.robot.rightDrive1, 1, this.kP, this.kI);
                } else {
                    this.robot.leftDrive1.set(ControlMode.PercentOutput, 0);
                    this.robot.rightDrive1.set(ControlMode.PercentOutput, 0);
                    vision.resetPID();
                }
            } else {
                vision.resetPID();
            }

        } catch (ConcurrentModificationException ignore) {
            // Just ignore these errors, but catch all others
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}