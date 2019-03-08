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

	double kP = 2, kI = 0.001, kD = 0, kF = 0, kG = 0.075;
	double multiplyLeft = 1.125;
	int iZone = 100;
	int cruiseVel = 200, maxAccel = 800;
	double arbFeedForward = 0;

	int forwardLimit = 90, backwardLimit = 90;

	int kTimeOutMs = 25;

	private RobotMap robot = new RobotMap();
	double wristSetPoint;
	int minus180 = -3218;
	int zero = -1170;
	double wristTicksPerDeg = ((double) (zero - minus180)) / 180;
	double straightUp = (minus180 + wristAngToTick(90));// -2190;

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
		SmartDashboard.putNumber("Left side multiplier", multiplyLeft);
		SmartDashboard.putNumber("iZone", iZone);
		SmartDashboard.putNumber("Cruise Vel", cruiseVel);
		SmartDashboard.putNumber("Max Accel", maxAccel);
		this.robot.wrist.configAllowableClosedloopError(0, 10);
		wristSetPoint = this.robot.wrist.getSelectedSensorPosition();
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
		multiplyLeft = SmartDashboard.getNumber("Left side multiplier", multiplyLeft);
		iZone = (int) SmartDashboard.getNumber("iZone", iZone);
		cruiseVel = (int) SmartDashboard.getNumber("Cruise Vel", 0);
		maxAccel = (int) SmartDashboard.getNumber("Max Accel", 0);
		this.robot.wrist.config_kP(0, kP, kTimeOutMs);
		this.robot.wrist.config_kI(0, kI, kTimeOutMs);
		this.robot.wrist.config_kD(0, kD, kTimeOutMs);
		this.robot.wrist.config_kF(0, kF, kTimeOutMs);
		this.robot.wrist.config_IntegralZone(0, iZone, kTimeOutMs);
		this.robot.wrist.configMotionCruiseVelocity(/* ticks per 100MS */(int) (cruiseVel), kTimeOutMs);
		this.robot.wrist.configMotionAcceleration(/* ticks per 100MS per second */(int) (maxAccel), kTimeOutMs);
		wristSetPoint = this.robot.wrist.getSelectedSensorPosition();
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

			SmartDashboard.putNumber("Error", this.robot.wrist.getClosedLoopError(0));
			SmartDashboard.putNumber("Wrist Output", this.robot.wrist.getMotorOutputPercent());
			SmartDashboard.putNumber("Wrist Velocity", this.robot.wrist.getSelectedSensorVelocity());
			SmartDashboard.putNumber("SetPoint", wristSetPoint);

			SmartDashboard.putNumber("Raw Wrist Position", this.robot.wrist.getPosition());
			SmartDashboard.putNumber("Adjusted Wrist Position (Ang)",
					wristTickToAng(this.robot.wrist.getPosition() - straightUp));
			SmartDashboard.putNumber("I Accumulation", this.robot.wrist.getIntegralAccumulator());
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
			double forward = this.robot.driver.getY(Hand.kLeft) * .8, turn = this.robot.driver.getX(Hand.kRight) * .5;
			if (Math.abs(forward) < 0.2d) {
				forward = 0;
			}
			if (Math.abs(turn) < 0.2d) {
				turn = 0;
			}
			// Basic west coast drive code
			if (!this.robot.driver.getStickButton(Hand.kLeft)) {
				this.robot.leftDrive.setPower((forward - turn) * .5 * multiplyLeft);
				this.robot.rightDrive.setPower((forward + turn) * .5);
			} else {
				this.robot.leftDrive.setPower((forward - turn) * multiplyLeft);
				this.robot.rightDrive.setPower((forward + turn));
			}
			if (forward == 0 && turn == 0) {
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();

			}

			// Outtake

			//If the sensor sees nothing
			if(this.robot.ballIn.get()){
				//op triggers override buttons
				if(this.robot.operator.getTriggerAxis(Hand.kLeft) > .1 || this.robot.operator.getTriggerAxis(Hand.kRight) > .1){
					this.robot.leftOuttake.setPower(this.robot.operator.getTriggerAxis(Hand.kLeft) - this.robot.operator.getTriggerAxis(Hand.kRight));
				}
				//op buttons
				else if(this.robot.operator.getBumper(Hand.kLeft)){
					this.robot.leftOuttake.setPower(.5);
				}
				else if(this.robot.operator.getBumper(Hand.kRight)){
					this.robot.leftOuttake.setPower(-.5);
				}
				//if no input on op, set to 0
				else{
					this.robot.leftOuttake.setPower(0);
				}
			}
			//if a ball is sensed only allow outtaking
			else{
				//Triggers override buttons
				if(this.robot.operator.getTriggerAxis(Hand.kRight) > .1){
					this.robot.leftOuttake.setPower(-this.robot.operator.getTriggerAxis(Hand.kRight));
				}
				//Buttons
				else if(this.robot.operator.getBumper(Hand.kLeft)){
					this.robot.leftOuttake.setPower(-.5);
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
			if (this.robot.operator.getPOV() != -1
					&& (this.robot.operator.getPOV() <= 90 || this.robot.operator.getPOV() >= 270)) {
				double angleInput = this.convertMinus180To180(this.robot.operator.getPOV());
				wristSetPoint = wristAngToTick(angleInput) + this.straightUp;
			}
			// Don't let desired position get out of hand (even if it does, talon won't let
			// motor keep going)
			if (wristSetPoint < (straightUp - wristAngToTick(backwardLimit))) {
				wristSetPoint = (straightUp - wristAngToTick(backwardLimit));
			}
			if (wristSetPoint > (straightUp + wristAngToTick(forwardLimit))) {
				wristSetPoint = (straightUp + wristAngToTick(forwardLimit));
			}
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
			if (this.robot.operator.getBButtonPressed()) {
				this.robot.togglePopper();
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