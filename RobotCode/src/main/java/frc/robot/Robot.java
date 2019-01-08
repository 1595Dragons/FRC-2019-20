/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private RobotMap robot;

	private Vision visionCode = new Vision();

	private SendableChooser<RobotType> robotType = new SendableChooser<RobotType>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		// Get the kind of robot
		robotType.setDefaultOption("2018 Chicago robot", RobotType.Chicago_2018);
		robotType.addOption("2018 China robot", RobotType.China_2018);
		SmartDashboard.putData(robotType);

		// Setup the camera
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(480,240);
		camera.setFPS(15);
		
	}

	@Override
	public void robotPeriodic() {

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

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {

		robot = new RobotMap(robotType.getSelected());
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		// Calculate drive code, with turbo button
		double forward = robot.gamepad1.getStickButton(Hand.kLeft) ? robot.gamepad1.getY(Hand.kLeft)
				: robot.gamepad1.getY(Hand.kLeft) / 2,
				turn = robot.gamepad1.getStickButton(Hand.kLeft) ? robot.gamepad1.getX(Hand.kRight)
						: robot.gamepad1.getX(Hand.kRight) / 2;

		if (Math.abs(forward) > 0.05d || Math.abs(turn) > 0.05d) {

			robot.leftDrive1.set(ControlMode.PercentOutput, forward - turn);
			robot.rightDrive1.set(ControlMode.PercentOutput, forward + turn);

		} else {
			robot.leftDrive1.set(ControlMode.PercentOutput, 0);
			robot.rightDrive1.set(ControlMode.PercentOutput, 0);
		}

		if (robotType.getSelected().equals(RobotType.China_2018)) {
			robot.liftDrive1.set(ControlMode.PercentOutput, robot.gamepad2.getY(Hand.kLeft));
		}

		visionCode.findTape();

		SmartDashboard.putNumber("Left drive power", robot.leftDrive1.getMotorOutputPercent());
		SmartDashboard.putNumber("Right drive power", robot.rightDrive1.getMotorOutputPercent());

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}