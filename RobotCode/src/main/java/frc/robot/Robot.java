/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	private RobotMap robot = new RobotMap();

	private TeleOp teleOp = new TeleOp(this.robot);

	private circlePath autonomous = new circlePath(this.robot);

	public static boolean tristanMode = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {
			// Set the endcoders to 0
			robot.rightDrive1.setSelectedSensorPosition(0);
			robot.leftDrive1.setSelectedSensorPosition(0);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void robotPeriodic() {
		try {
			SmartDashboard.putNumber("Left position", robot.leftDrive1.getSelectedSensorPosition());
			SmartDashboard.putNumber("Right position", robot.rightDrive1.getSelectedSensorPosition());

			// Check for tristan mode
			if (this.robot.gamepad1.getStartButtonPressed()) {
				Robot.tristanMode = !Robot.tristanMode;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		try {
			autonomous.init();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		try {
			autonomous.periodic();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void teleopInit() {
		try {
			teleOp.init();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void teleopPeriodic() {
		try {
			teleOp.periodic();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void testInit() {

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}

}