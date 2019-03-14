/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	boolean manualOveride = false;
	double kP = 2, kI = 0.001, kD = 0, kF = 0, kG = 0.075;
	double DTkP = 2, DTkI = 0.003, DTkD = 0, LDTkF = 2, RDTkF = 2;
	int iZone = 100;
	int cruiseVel = 200, maxAccel = 800;
	double maxVelDT = 400;
	double arbFeedForward = 0;

	double outtakePresetSpeed = .65;

	int forwardLimit = 90, backwardLimit = 90;

	int kTimeOutMs = 25;

	private RobotMap robot = new RobotMap();
	double wristSetPoint;
	
	double wristTicksPerDeg = 2048 / 180;
	double exchangePosOffset = 200;

	double zero;
	double minus180;
	double straightUp;


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
		// PID for wrist
		this.robot.setupTestMode();
		SmartDashboard.putNumber("kP", kP);
		SmartDashboard.putNumber("kI", kI);
		SmartDashboard.putNumber("kD", kD);
		SmartDashboard.putNumber("kF", kF);
		SmartDashboard.putNumber("kG", kG);
		SmartDashboard.putNumber("DTkP", DTkP);
		SmartDashboard.putNumber("DTkI", DTkI);
		SmartDashboard.putNumber("DTkD", DTkD);
		SmartDashboard.putNumber("LDTkF", LDTkF);
		SmartDashboard.putNumber("RDTkF", RDTkF);
		SmartDashboard.putNumber("Outtake Preset Speed", outtakePresetSpeed);
		SmartDashboard.putNumber("iZone", iZone);
		SmartDashboard.putNumber("Cruise Vel", cruiseVel);
		SmartDashboard.putNumber("Max Accel", maxAccel);
		SmartDashboard.putNumber("Max Vel DT", maxVelDT);
		this.robot.wrist.configAllowableClosedloopError(0, 10);
		wristSetPoint = this.robot.wrist.getSelectedSensorPosition();

		if(this.robot.PRACTICEBOT){
			zero = 1874;
		}
		else{
			zero = -2045;
		}
		minus180 = zero-2048;
		straightUp = (zero + minus180) / 2;
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
		this.robot.nothing1.set(true);
		this.robot.nothing2.set(true);

		// Limits for the wrist
		this.robot.wrist.configForwardSoftLimitThreshold((int) (straightUp + wristAngToTick(forwardLimit)));
		this.robot.wrist.configReverseSoftLimitThreshold((int) (straightUp - wristAngToTick(backwardLimit)));
		this.robot.wrist.configForwardSoftLimitEnable(true);
		this.robot.wrist.configReverseSoftLimitEnable(true);

		// PIDF && Motion Profile constants / configs
		kP = SmartDashboard.getNumber("kP", kP);
		kI = SmartDashboard.getNumber("kI", kI);
		kD = SmartDashboard.getNumber("kD", kD);
		kF = SmartDashboard.getNumber("kF", kF);
		kG = SmartDashboard.getNumber("kG", kG);
		DTkP = SmartDashboard.getNumber("DTkP", DTkP);
		DTkI = SmartDashboard.getNumber("DTkI", DTkI);
		DTkD = SmartDashboard.getNumber("DTkD", DTkD);
		LDTkF = SmartDashboard.getNumber("LDTkF", LDTkF);
		RDTkF = SmartDashboard.getNumber("RDTkF", RDTkF);
		outtakePresetSpeed = SmartDashboard.getNumber("Outtake Preset Speed", outtakePresetSpeed);
		iZone = (int) SmartDashboard.getNumber("iZone", iZone);
		cruiseVel = (int) SmartDashboard.getNumber("Cruise Vel", cruiseVel);
		maxAccel = (int) SmartDashboard.getNumber("Max Accel", maxAccel);
		maxVelDT = SmartDashboard.getNumber("Max Vel DT", maxVelDT);
		this.robot.wrist.config_kP(0, kP, kTimeOutMs);
		this.robot.wrist.config_kI(0, kI, kTimeOutMs);
		this.robot.wrist.config_kD(0, kD, kTimeOutMs);
		this.robot.wrist.config_kF(0, kF, kTimeOutMs);
		this.robot.wrist.config_IntegralZone(0, iZone, kTimeOutMs);
		this.robot.wrist.configMotionCruiseVelocity(/* ticks per 100MS */(int) (cruiseVel), kTimeOutMs);
		this.robot.wrist.configMotionAcceleration(/* ticks per 100MS per second */(int) (maxAccel), kTimeOutMs);
		wristSetPoint = this.robot.wrist.getSelectedSensorPosition();
		this.robot.leftDrive.config_kP(0, DTkP);
		this.robot.leftDrive.config_kI(0, DTkI);
		this.robot.leftDrive.config_kD(0, DTkD);
		this.robot.leftDrive.config_kF(0, LDTkF);
		this.robot.rightDrive.config_kP(0, DTkP);
		this.robot.rightDrive.config_kI(0, DTkI);
		this.robot.rightDrive.config_kD(0, DTkD);
		this.robot.rightDrive.config_kF(0, RDTkF);
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
			SmartDashboard.putNumber("Left velocity", this.robot.leftDrive.getSelectedSensorVelocity());
			SmartDashboard.putNumber("Right velocity", this.robot.rightDrive.getSelectedSensorVelocity());
			SmartDashboard.putNumber("Left Position", this.robot.leftDrive.getSelectedSensorPosition());
			SmartDashboard.putNumber("Right Position", this.robot.rightDrive.getSelectedSensorPosition());


			SmartDashboard.putNumber("Error", this.robot.wrist.getClosedLoopError(0));
			SmartDashboard.putNumber("Wrist Output", this.robot.wrist.getMotorOutputPercent());
			SmartDashboard.putNumber("Wrist Velocity", this.robot.wrist.getSelectedSensorVelocity());
			SmartDashboard.putNumber("SetPoint", wristSetPoint);

			SmartDashboard.putNumber("Raw Wrist Position", this.robot.wrist.getPosition());
			SmartDashboard.putNumber("Adjusted Wrist Position (Ang)",
					wristTickToAng(this.robot.wrist.getPosition() - straightUp));
			SmartDashboard.putNumber("I Accumulation", this.robot.wrist.getIntegralAccumulator());

			SmartDashboard.putBoolean("Has ball", !this.robot.ballIn.get());
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
			double forward = Math.pow(this.robot.driver.getY(Hand.kLeft), 1) * .8, turn = Math.pow(this.robot.driver.getX(Hand.kRight), 1) * .6;
			if (Math.abs(forward) < 0.2d) {
				forward = 0;
			}
			if (Math.abs(turn) < 0.2d) {
				turn = 0;
			}
			// Basic west coast drive code
			/*if (!this.robot.driver.getStickButton(Hand.kLeft)) {
				this.robot.leftDrive.setPower((forward - turn) * .5 * multiplyLeft);
				this.robot.rightDrive.setPower((forward + turn) * .5);
			} else {
				this.robot.leftDrive.setPower((forward - turn) * multiplyLeft);
				this.robot.rightDrive.setPower((forward + turn));
			}
			if (forward == 0 && turn == 0) {
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}*/
			if (!this.robot.driver.getStickButton(Hand.kLeft)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * .4 * maxVelDT);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * .4 * maxVelDT);
				SmartDashboard.putNumber("lVel", (forward - turn) * .4 * maxVelDT);
				SmartDashboard.putNumber("lerror", this.robot.leftDrive.getClosedLoopError());
				SmartDashboard.putNumber("rVel", (forward + turn) * .4 * maxVelDT);
				SmartDashboard.putNumber("rerror", this.robot.rightDrive.getClosedLoopError());
			} else {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * maxVelDT);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * maxVelDT);
			}
			if (forward == 0 && turn == 0) {
				this.robot.leftDrive.set(ControlMode.Velocity, 0);
				this.robot.rightDrive.set(ControlMode.Velocity, 0);
			}

			// Outtake

			//If the sensor sees nothing
			if(this.robot.ballIn.get()){
				if(this.robot.operator.getTriggerAxis(Hand.kLeft) > .1 || this.robot.operator.getTriggerAxis(Hand.kRight) > .1){
					this.robot.leftOuttake.setPower((this.robot.operator.getTriggerAxis(Hand.kLeft) - this.robot.operator.getTriggerAxis(Hand.kRight)) * outtakePresetSpeed);
				}
				//if no input on op, set to 0
				else{
					this.robot.leftOuttake.setPower(0);
				}
			}
			//if a ball is sensed only allow outtaking
			else{
				if(this.robot.operator.getTriggerAxis(Hand.kRight) > .1){
					this.robot.leftOuttake.setPower((-this.robot.operator.getTriggerAxis(Hand.kRight)) * outtakePresetSpeed);
				}
				//Stop
				else{
					this.robot.leftOuttake.setPower(0);
				}
			}


			// Wrist
			// Update desired position
			if (Math.abs(this.robot.operator.getY(Hand.kLeft)) > .2d) {
				wristSetPoint += this.robot.operator.getY(Hand.kLeft) * 20;
			}
			// Gets d-pad inputs
			/*if (this.robot.operator.getPOV() == 0 || this.robot.operator.getPOV() == 90 || this.robot.operator.getPOV() == 270) {
				double angleInput = this.convertMinus180To180(this.robot.operator.getPOV());
				wristSetPoint = wristAngToTick(angleInput) + this.straightUp;
			}*/
			if(this.robot.operator.getPOV() == 0){ //straight up
				wristSetPoint = straightUp;
			}
			else if(this.robot.operator.getPOV() == 90){ //minus 180
				wristSetPoint = straightUp-1024;
			}
			else if(this.robot.operator.getPOV() == 270){ //zero
				wristSetPoint = straightUp+1024;
			}
			else if(this.robot.operator.getBumper(Hand.kRight)){
				wristSetPoint = straightUp - exchangePosOffset;
			}
			else if(this.robot.operator.getBumper(Hand.kLeft)){
				wristSetPoint = straightUp + exchangePosOffset;
			}
			if(this.robot.operator.getPOV() >= 0 && this.robot.operator.getPOV() <= 180){
				this.robot.limeLightServo.setAngle(this.robot.operator.getPOV());
			}
			// Don't let desired position get out of hand (even if it does, talon won't let
			// motor keep going)
			/*if (wristSetPoint < (straightUp - wristAngToTick(backwardLimit))) {
				wristSetPoint = (straightUp - wristAngToTick(backwardLimit));
			}
			if (wristSetPoint > (straightUp + wristAngToTick(forwardLimit))) {
				wristSetPoint = (straightUp + wristAngToTick(forwardLimit));
			}*/
			// arbitrary feed forward accounts for gravity
			arbFeedForward = -Math
					.sin(wristTickToAng(this.robot.wrist.getSelectedSensorPosition() - straightUp) * (Math.PI / 180))
					* kG;
			this.robot.wrist.set(ControlMode.MotionMagic, wristSetPoint, DemandType.ArbitraryFeedForward,
					arbFeedForward);

			// Hatch mechanism
			if (this.robot.operator.getAButtonPressed()) {
				this.robot.toggleHatchMechanism();
			}
			if(this.robot.operator.getBButton()){

			}
			if (this.robot.operator.getXButtonPressed()) {
				this.robot.toggleHatchExtension();
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public double convertMinus180To180(double theta) {
		double a = theta;
		while (a > 180 || a < -180) {
			if (a > 180) {
				a -= 360;
			} else if (a < -180) {
				a += 360;
			}
		}
		return a;
	}

	public double wristAngToTick(double theta) {
		return theta * wristTicksPerDeg;
	}

	public double wristTickToAng(double tick) {
		return tick / wristTicksPerDeg;
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