package frc.robot;

import java.util.ConcurrentModificationException;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is where we put the code that we want to run during teleop.
 * 
 * In the event that either <code>init()</code> or <code>periodic()</code> throw
 * an error, this can be caught on the Robot.java end witha simple
 * <code>try{}catch{}</code> block.
 */
public class TeleOp {

    private XboxController driver;

    private RobotMap robot;

    // For using a PID, centerX is our error
    private double kP, kI, kD;
    private final double initialKP = 0.02d, initialKI = 4.0E-4d, initialKD = 0.0d;

    // Get the status of the vision stick
    private boolean canSeeTarget;

    private Tracking tracking;

    private double centerX;

    public TeleOp(RobotMap robot) {
        this.robot = robot;
    }

    /**
     * This is code that we only want to run <b>once</b>.
     */
    public void init() {
        SmartDashboard.putNumber("kP", this.initialKP);
        SmartDashboard.putNumber("kI", this.initialKI);
        SmartDashboard.putNumber("kD", this.initialKD);
        tracking = new Tracking(kP, kI, kD);
        System.out.println("Cos of 90: " + Math.cos(90));
        this.driver = this.robot.gamepad1;
    }

    /**
     * This is for code that we want to run <b>repeatidly</b>.
     */
    public void periodic() {

        boolean visionTracking;
        if (this.robot.vision.isRunning) {
            visionTracking = SmartDashboard.getBoolean("Vision tracking", false);
        } else {
            visionTracking = false;
        }

        // Toggle vision tracking, but only if vision is running
        if (this.robot.vision.isRunning) {
            if (this.driver.getBumperPressed(Hand.kLeft) || this.driver.getBumperPressed(Hand.kRight)) {
                SmartDashboard.putBoolean("Vision tracking", !SmartDashboard.getBoolean("Vision tracking", false));
            }
        }

        // Determine the drive methods
        if (visionTracking && this.robot.vision.isRunning) {
            this.visionTrackDrive();
        } else {
            this.westCoastDrive();
            this.driver.setRumble(RumbleType.kRightRumble, 0.0d);
            this.driver.setRumble(RumbleType.kLeftRumble, 0.0d);
        }

        // Hatch mechanism
        if (this.driver.getTriggerAxis(Hand.kLeft) > 0.1d) {
            this.robot.secureHatchPanel();
        }
        if (this.driver.getTriggerAxis(Hand.kRight) > 0.1d) {
            this.robot.releaseHatchPanel();
        }
        if (this.driver.getAButtonPressed()) {
            this.robot.toggleHatchMechanism();
        }

        // Get whether or not the robot can see the target
        this.canSeeTarget = this.robot.vision.numberOfTargets() != 0;
        SmartDashboard.putBoolean("Can see vision stick", canSeeTarget);

        if (this.robot.vision.isRunning) {
            if (this.canSeeTarget) {
                this.centerX = this.robot.vision.findCenterX();
                SmartDashboard.putNumber("Degrees", this.robot.vision.getDegree(centerX));
                SmartDashboard.putNumber("Width", this.robot.vision.getTargetWidth());
                SmartDashboard.putNumber("Distance", this.robot.vision.getDistance(centerX));
            }
        }

        SmartDashboard.putNumber("Left drive 1 power", this.robot.leftDrive1.getMotorOutputPercent());
        SmartDashboard.putNumber("Right drive 1 power", this.robot.rightDrive1.getMotorOutputPercent());

    }

    private void westCoastDrive() {

        // Calculate drive code, with turbo button
        double forward = this.driver.getStickButton(Hand.kLeft) ? this.driver.getY(Hand.kLeft)
                : this.driver.getY(Hand.kLeft) / 2,
                turn = this.driver.getStickButton(Hand.kLeft) ? this.driver.getX(Hand.kRight)
                        : this.driver.getX(Hand.kRight) / 2;

        // Check if there should be turning percision
        if (this.driver.getStickButton(Hand.kRight)) {
            turn /= 2;
        }

        // Check if tristan mode is enabled
        if (Robot.tristanMode) {
            forward = forward / 4;
            turn = turn / 4;
        }

        // Basic west coast drive code
        if (Math.abs(forward) > 0.025d || Math.abs(turn) > 0.025d) {
            this.robot.leftDrive1.set(ControlMode.PercentOutput, forward - turn);
            this.robot.rightDrive1.set(ControlMode.PercentOutput, forward + turn);
        } else {
            this.robot.leftDrive1.set(ControlMode.PercentOutput, 0);
            this.robot.rightDrive1.set(ControlMode.PercentOutput, 0);
        }

    }

    private void visionTrackDrive() {
        try {

            double pidPower;
            if (this.canSeeTarget) {

                this.robot.gamepad1.setRumble(RumbleType.kRightRumble, 0.5d);
                this.robot.gamepad1.setRumble(RumbleType.kLeftRumble, 0.5d);

                // Check for a change in PID values
                if (SmartDashboard.getNumber("kP", this.initialKP) != this.kP) {
                    System.out.printf("Updating kP from %s to %s\n", this.kP,
                            SmartDashboard.getNumber("kP", this.initialKP));
                    this.kP = SmartDashboard.getNumber("kP", this.initialKP);
                    tracking.pid.setP(this.kP);
                }

                if (SmartDashboard.getNumber("kI", this.initialKI) != this.kI) {
                    System.out.printf("Updating kI from %s to %s\n", this.kI,
                            SmartDashboard.getNumber("kI", this.initialKI));
                    this.kI = SmartDashboard.getNumber("kI", this.initialKI);
                    tracking.pid.setI(this.kI);
                }

                if (SmartDashboard.getNumber("kD", this.initialKD) != this.kD) {
                    System.out.printf("Updating kD from %s to %s\n", this.kD,
                            SmartDashboard.getNumber("kD", this.initialKD));
                    this.kD = SmartDashboard.getNumber("kD", this.initialKD);
                    tracking.pid.setD(this.kD);
                }

                if (Math.abs(this.centerX) > 2) {
                    pidPower = tracking.trackTurnPower(this.robot.vision.getDegree(this.centerX));
                } else {
                    pidPower = 0;
                    tracking.pid.reset();
                }
            } else {
                this.robot.gamepad1.setRumble(RumbleType.kRightRumble, 0.25d);
                this.robot.gamepad1.setRumble(RumbleType.kLeftRumble, 0.25d);
                pidPower = 0;
                tracking.pid.reset();
            }

            this.robot.leftDrive1.set(ControlMode.PercentOutput, this.robot.gamepad1.getY(Hand.kLeft) / 2 + pidPower);
            this.robot.rightDrive1.set(ControlMode.PercentOutput, this.robot.gamepad1.getY(Hand.kLeft) / 2 - pidPower);

        } catch (ConcurrentModificationException ignore) {
            // Just ignore these errors, but catch all others
        } catch (Exception e) {
            // Disable vision tracking
            SmartDashboard.putBoolean("Vision tracking", false);
            e.printStackTrace();
        }
    }

}