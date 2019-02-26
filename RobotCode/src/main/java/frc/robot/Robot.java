/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	double kP = 0.0025, kI = 0, kD = .001, kF = 0, 
	int cruiseVel = 100, maxAccel = 800;

	int kTimeOutMs = 25;

	private RobotMap robot = new RobotMap();

	double wristTicksPerDeg = 2048/180;
	double wristPos;
	int straightUp;
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
		//PID for wrist
		this.robot.setupTestMode();
		SmartDashboard.putNumber("kP", kP);
		SmartDashboard.putNumber("kI", kI);
		SmartDashboard.putNumber("kD", kD);
		SmartDashboard.putNumber("kF", kF);
		SmartDashboard.putNumber("Cruise Vel", cruiseVel);
		SmartDashboard.putNumber("Max Accel", maxAccel);
		wristPos = this.robot.wrist.getSelectedSensorPosition();
		// Limits for the wrist
		straightUp = -150;
		this.robot.wrist.configForwardSoftLimitThreshold((int) (straightUp + 85 * this.wristTicksPerDeg));
		this.robot.wrist.configReverseSoftLimitThreshold((int) (straightUp - 85 * this.wristTicksPerDeg));
		this.robot.wrist.configForwardSoftLimitEnable(true);
		this.robot.wrist.configReverseSoftLimitEnable(true);
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
		kP = SmartDashboard.getNumber("kP", 0);
		kI = SmartDashboard.getNumber("kI", 0);
		kD = SmartDashboard.getNumber("kD", 0);
		kF = SmartDashboard.getNumber("kF", 0);
		cruiseVel = (int) SmartDashboard.getNumber("Cruise Vel", 0);
		maxAccel = (int) SmartDashboard.getNumber("Max Accel", 0);
		this.robot.wrist.config_kP(0, kP, kTimeOutMs);
		this.robot.wrist.config_kI(0, kI, kTimeOutMs);
		this.robot.wrist.config_kD(0, kD, kTimeOutMs);
		this.robot.wrist.config_kF(0, kF, kTimeOutMs);
		this.robot.wrist.configMotionCruiseVelocity(/*ticks per 100MS*/cruiseVel, kTimeOutMs);//for 180 deg travel in 2s
		this.robot.wrist.configMotionAcceleration(/*ticks per 100MS per second*/maxAccel, kTimeOutMs);//for max velocity in .125s
		wristPos = this.robot.wrist.getSelectedSensorPosition();
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
			SmartDashboard.putNumber("Voltage", this.robot.wrist.getMotorOutputVoltage());
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
			double forward = this.robot.driver.getY(Hand.kLeft) * .8, turn = this.robot.driver.getX(Hand.kRight) * .4;
				//	+ this.robot.driver.getTriggerAxis(Hand.kRight) - this.robot.driver.getTriggerAxis(Hand.kLeft);
			if (Math.abs(forward) < 0.2d) {
				forward = 0;
			}
			/*if (Math.abs(turn) < 0.2d) {
				turn = this.robot.driver.getTriggerAxis(Hand.kRight) - this.robot.driver.getTriggerAxis(Hand.kLeft);
			}*/

			// Basic west coast drive code
			if (!this.robot.driver.getStickButton(Hand.kLeft)) {
				this.robot.leftDrive.setPower((forward - turn) * .5);
				this.robot.rightDrive.setPower((forward + turn) * .5);
			} else {
				this.robot.leftDrive.setPower((forward - turn));
				this.robot.rightDrive.setPower((forward + turn));
			}
			if (forward == 0 && turn == 0) {
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}

			// Intake
			this.robot.leftIntake.setPower(
					(this.robot.operator.getTriggerAxis(Hand.kLeft) - this.robot.operator.getTriggerAxis(Hand.kRight)));

			// Wrist
			//Update desired position
			if(Math.abs(this.robot.operator.getY(Hand.kLeft)) > .2d){
				wristPos += this.robot.operator.getY(Hand.kLeft) * 100;
			}
			//Gets d-pad inputs
			if(this.robot.operator.getPOV() != -1 && (this.robot.operator.getPOV() < 90 || this.robot.operator.getPOV() > 270)){
				double angleInput = this.convertMinus180To180(this.robot.operator.getPOV());
				wristPos = (angleInput * wristTicksPerDeg) + this.straightUp;
			}
			//Don't let desired position get out of hand (even if it does, talon won't let motor keep going)
			if(wristPos < (straightUp - 85 * this.wristTicksPerDeg)){
				wristPos = (straightUp - 85 * this.wristTicksPerDeg);
			}
			if(wristPos > (straightUp + 85 * this.wristTicksPerDeg)){
				wristPos = (straightUp + 85 * this.wristTicksPerDeg);
			}
			//this.robot.wrist.driveToPosition(wristPos);
			this.robot.wrist.set(ControlMode.MotionMagic, wristPos);
			SmartDashboard.putNumber("SetPoint", wristPos);

			// Hatch mechanism
			if (this.robot.operator.getBumper(Hand.kLeft)) {
				this.robot.extendHatch();
			}
			if (this.robot.operator.getBumper(Hand.kRight)) {
				this.robot.retracthHatch();
			}
			if (this.robot.operator.getAButtonPressed()) {
				this.robot.toggleHatchMechanism();
			}
			if (this.robot.operator.getBButtonPressed()) {
				this.robot.togglePopper();
			}
			if (this.robot.operator.getXButton()) {
				this.robot.toggleHatchExtension();
			}

			// Display the wrist position
			SmartDashboard.putNumber("Wrist Position", this.robot.wrist.getPosition());
			SmartDashboard.putNumber("Wrist Position (Ang)", this.robot.wrist.getPosition() / this.wristTicksPerDeg);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public double convertMinus180To180(double theta){
		double a = 0;
		while(a > 180 || a < -180){
			if(a > 180){
				a -= 360;
			}
			else if(a < -180){
				a -= 360;
			}
		}
		return a;
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