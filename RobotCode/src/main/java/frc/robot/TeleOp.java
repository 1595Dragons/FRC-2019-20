package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

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

    public TeleOp(RobotMap robot) {
        this.robot = robot;
    }

    /**
     * This is code that we only want to run <b>once</b>.
     */
    public void init() {
        this.driver = this.robot.gamepad1;
    }

    /**
     * This is for code that we want to run <b>repeatidly</b>.
     */
    public void periodic() {

        // Calculate drive power
        double forward = this.driver.getY(Hand.kLeft), turn = this.driver.getX(Hand.kRight);

        // Basic west coast drive code
        if (Math.abs(forward) > 0.05d || Math.abs(turn) > 0.05d) {
            this.robot.leftDrive.setPower(forward - turn);
            this.robot.rightDrive.setPower(forward + turn);
        } else {
            this.robot.leftDrive.stop();
            this.robot.rightDrive.stop();
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

    }
}