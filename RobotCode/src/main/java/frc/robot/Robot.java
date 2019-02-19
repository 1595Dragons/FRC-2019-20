/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private RobotMap robot = new RobotMap();

	/**
	 * Change the update frequency to 0.04 seconds (40 ms) in order silence the
	 * watch dog...
	 * 
	 * Man that sounds bad :(
	 */
	public Robot() {
		super(0.04d);
	}

	/**
	 * Robot-wide initialization code should go here.
	 *
	 * <p>
	 * Users should override this method for default Robot-wide initialization which
	 * will be called when the robot is first powered on. It will be called exactly
	 * one time.
	 *
	 * <p>
	 * Warning: the Driver Station "Robot Code" light and FMS "Robot Ready"
	 * indicators will be off until RobotInit() exits. Code in RobotInit() that
	 * waits for enable will cause the robot to never indicate that the code is
	 * ready, causing the robot to be bypassed in a match.
	 */
	@Override
	public void robotInit() {
		this.robot.setupTestMode();
	}

	/**
	 * Initialization code for disabled mode should go here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be
	 * called each time the robot enters disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	/**
	 * Initialization code for autonomous mode should go here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be
	 * called each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
	}

	/**
	 * Initialization code for teleop mode should go here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be
	 * called each time the robot enters teleop mode.
	 */
	@Override
	public void teleopInit() {
		//Limits for the wrist
		this.robot.wrist.configForwardSoftLimitThreshold(-3797);
		this.robot.wrist.configReverseSoftLimitThreshold(-4797);
		this.robot.wrist.configForwardSoftLimitEnable(true);
		this.robot.wrist.configReverseSoftLimitEnable(true);
	}

	/**
	 * Initialization code for test mode should go here.
	 *
	 * <p>
	 * Users should override this method for initialization code which will be
	 * called each time the robot enters test mode.
	 */
	@Override
	public void testInit() {
	}

	/**
	 * Periodic code for all robot modes should go here.
	 */
	@Override
	public void robotPeriodic() {
		try {
			SmartDashboard.putNumber("Left position", this.robot.leftDrive.getPosition());
			SmartDashboard.putNumber("Right position", this.robot.rightDrive.getPosition());

			SmartDashboard.putNumber("Left power", this.robot.leftDrive.getMotorOutputPercent());
			SmartDashboard.putNumber("Rigth power", this.robot.rightDrive.getMotorOutputPercent());
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Periodic code for disabled mode should go here.
	 */
	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

	/**
	 * Periodic code for autonomous mode should go here.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	/**
	 * Periodic code for teleop mode should go here.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			// Calculate drive power
			double forward = this.robot.driver.getY(Hand.kLeft), turn = this.robot.driver.getX(Hand.kRight);

			// Basic west coast drive code
			if (Math.abs(forward) > 0.1d || Math.abs(turn) > 0.1d) {
				this.robot.leftDrive.setPower(Math.pow((forward - turn), 3) * .5);
				this.robot.rightDrive.setPower(Math.pow((forward + turn), 3) * .5);
			} else {
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}

			//Intake
			this.robot.leftIntake.setPower((this.robot.operator.getTriggerAxis(Hand.kLeft) - this.robot.operator.getTriggerAxis(Hand.kRight)));

			//Wrist
			double wristPower = this.robot.operator.getY(Hand.kLeft) * .5;
			this.robot.wrist.setPower(wristPower);

			// Hatch mechanism
			if (this.robot.driver.getTriggerAxis(Hand.kLeft) > 0.1d) {
				this.robot.secureHatchPanel();
			}
			if (this.robot.driver.getTriggerAxis(Hand.kRight) > 0.1d) {
				this.robot.releaseHatchPanel();
			}
			if (this.robot.driver.getAButtonPressed()) {
				this.robot.toggleHatchMechanism();
			}
			SmartDashboard.putNumber("Wrist Position", this.robot.wrist.getSelectedSensorPosition());

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Periodic code for test mode should go here.
	 */
	@Override
	public void testPeriodic() {
		try {
			this.robot.testMotors();
		} catch (Exception error) {
			error.printStackTrace();
		}
	}

}