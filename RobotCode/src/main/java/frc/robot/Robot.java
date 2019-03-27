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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Extenders;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private boolean neg = true, manualOveride = false, visEnabled = false, manualWrist = false;

	private double kP = 2, kI = 0.001, kD = 0, kF = 0, kG = 0.075, viskP = 4, viskI = 0.01, viskD = 0, DTkP = 5,
			DTkI = 0.003, DTkD = 0, LDTkF = 1, RDTkF = 1, maxVelDT = 400, arbFeedForward = 0, vis = 0, visSetpoint = 3,
			wristSetPoint, outtakePresetSpeed = .55, wristTicksPerDeg = 2048 / 180, exchangePosOffset = 200, zero,
			minus180, straightUp;

	private int iZone = 100, cruiseVel = 200, maxAccel = 800, forwardLimit = 90, backwardLimit = 90, kTimeOutMs = 25;

	private MiniPID pid = new MiniPID(viskP, viskI, viskD);

	private RobotMap robot = new RobotMap();

	// Subsystems for command based programming
	public static Extenders extender;

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

		// Setup individual drive motor tests
		this.robot.setupTestMode();

		Robot.extender = new Extenders();

		// PID for wrist
		SmartDashboard.putNumber("kP", this.kP);
		SmartDashboard.putNumber("viskP", this.viskP);
		SmartDashboard.putNumber("viskI", this.viskI);
		SmartDashboard.putNumber("viskD", this.viskD);
		SmartDashboard.putNumber("kI", this.kI);
		SmartDashboard.putNumber("kD", this.kD);
		SmartDashboard.putNumber("kF", this.kF);
		SmartDashboard.putNumber("kG", this.kG);
		SmartDashboard.putNumber("DTkP", this.DTkP);
		SmartDashboard.putNumber("DTkI", this.DTkI);
		SmartDashboard.putNumber("DTkD", this.DTkD);
		SmartDashboard.putNumber("LDTkF", this.LDTkF);
		SmartDashboard.putNumber("RDTkF", this.RDTkF);
		SmartDashboard.putNumber("Outtake Preset Speed", this.outtakePresetSpeed);
		SmartDashboard.putNumber("iZone", this.iZone);
		SmartDashboard.putNumber("Cruise Vel", this.cruiseVel);
		SmartDashboard.putNumber("Max Accel", this.maxAccel);
		SmartDashboard.putNumber("Max Vel DT", this.maxVelDT);
		SmartDashboard.putBoolean("neg", neg);
		this.robot.wrist.configAllowableClosedloopError(0, 10);
		this.wristSetPoint = this.robot.wrist.getPosition();

		// Try to setup limelight
		if (this.robot.limelight != null) {
			SmartDashboard.putBoolean("LED", true);
		}

		if (this.robot.PRACTICEBOT) {
			this.zero = 1874;
		} else {
			this.zero = -2063;
		}

		this.minus180 = this.zero - 2048;
		this.straightUp = (this.zero + this.minus180) / 2;
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
		Scheduler.getInstance().removeAll();
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
		this.teleopInit();
		// Close the hatch stuff
		this.robot.secureHatchPanel();
		/*
		 * if(this.robot.wrist.getSelectedSensorPosition() > 0){ neg = false; } else{
		 * neg = true; }
		 */
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
		this.robot.wrist.configForwardSoftLimitThreshold((int) (this.straightUp + wristAngToTick(this.forwardLimit)));
		this.robot.wrist.configReverseSoftLimitThreshold((int) (this.straightUp - wristAngToTick(this.backwardLimit)));
		this.robot.wrist.configForwardSoftLimitEnable(true);
		this.robot.wrist.configReverseSoftLimitEnable(true);

		// Limelight pid
		this.viskP = SmartDashboard.getNumber("viskP", this.viskP);
		this.viskI = SmartDashboard.getNumber("viskI", this.viskI);
		this.viskD = SmartDashboard.getNumber("viskD", this.viskD);
		this.pid.setP(this.viskP);
		this.pid.setI(this.viskI);
		this.pid.setD(this.viskD);

		// PIDF && Motion Profile constants / configs for the wrist
		this.kP = SmartDashboard.getNumber("kP", this.kP);
		this.kI = SmartDashboard.getNumber("kI", this.kI);
		this.kD = SmartDashboard.getNumber("kD", this.kD);
		this.kF = SmartDashboard.getNumber("kF", this.kF);
		this.kG = SmartDashboard.getNumber("kG", this.kG);
		this.iZone = (int) SmartDashboard.getNumber("iZone", this.iZone);
		this.cruiseVel = (int) SmartDashboard.getNumber("Cruise Vel", this.cruiseVel);
		this.maxAccel = (int) SmartDashboard.getNumber("Max Accel", this.maxAccel);
		this.maxVelDT = SmartDashboard.getNumber("Max Vel DT", this.maxVelDT);
		this.robot.wrist.config_kP(0, this.kP, this.kTimeOutMs);
		this.robot.wrist.config_kI(0, this.kI, this.kTimeOutMs);
		this.robot.wrist.config_kD(0, this.kD, this.kTimeOutMs);
		this.robot.wrist.config_kF(0, this.kF, this.kTimeOutMs);
		this.robot.wrist.config_IntegralZone(0, this.iZone, this.kTimeOutMs);
		this.robot.wrist.configMotionCruiseVelocity(/* ticks per 100MS */(int) (this.cruiseVel), this.kTimeOutMs);
		this.robot.wrist.configMotionAcceleration(/* ticks per 100MS per second */(int) (this.maxAccel),
				this.kTimeOutMs);
		this.wristSetPoint = this.robot.wrist.getPosition();
		this.robot.wrist.stop();

		// Drive train PID
		this.DTkP = SmartDashboard.getNumber("DTkP", this.DTkP);
		this.DTkI = SmartDashboard.getNumber("DTkI", this.DTkI);
		this.DTkD = SmartDashboard.getNumber("DTkD", this.DTkD);
		this.LDTkF = SmartDashboard.getNumber("LDTkF", this.LDTkF);
		this.RDTkF = SmartDashboard.getNumber("RDTkF", this.RDTkF);
		this.robot.leftDrive.config_kP(0, this.DTkP);
		this.robot.leftDrive.config_kI(0, this.DTkI);
		this.robot.leftDrive.config_kD(0, this.DTkD);
		this.robot.leftDrive.config_kF(0, this.LDTkF);
		this.robot.rightDrive.config_kP(0, this.DTkP);
		this.robot.rightDrive.config_kI(0, this.DTkI);
		this.robot.rightDrive.config_kD(0, this.DTkD);
		this.robot.rightDrive.config_kF(0, this.RDTkF);

		// Outtake speed limit
		this.outtakePresetSpeed = SmartDashboard.getNumber("Outtake Preset Speed", this.outtakePresetSpeed);

		if (this.robot.hatchPanelSecured) {
			this.robot.secureHatchPanel();
		} else {
			this.robot.releaseHatchPanel();
		}
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

			if (this.robot.hatchPanelSecured) {
				SmartDashboard.putString("Mittens", "Closed");
			} else {
				SmartDashboard.putString("Mittens", "Open");
			}

			// Drive train velocities
			SmartDashboard.putNumber("Left velocity", this.robot.leftDrive.getSelectedSensorVelocity());
			SmartDashboard.putNumber("Right velocity", this.robot.rightDrive.getSelectedSensorVelocity());

			// Zero
			SmartDashboard.putNumber("Zero", this.zero);

			// Log Buttons
			SmartDashboard.putBoolean("opA", this.robot.operator.getAButton());
			SmartDashboard.putBoolean("opB", this.robot.operator.getBButton());
			SmartDashboard.putBoolean("opX", this.robot.operator.getXButton());
			SmartDashboard.putBoolean("opY", this.robot.operator.getYButton());
			SmartDashboard.putNumber("opPOV", this.robot.operator.getPOV());
			SmartDashboard.putBoolean("opLB", this.robot.operator.getBumper(Hand.kLeft));
			SmartDashboard.putBoolean("oRLB", this.robot.operator.getBumper(Hand.kRight));
			SmartDashboard.putNumber("opLTr", this.robot.operator.getTriggerAxis(Hand.kLeft));
			SmartDashboard.putNumber("opRTr", this.robot.operator.getTriggerAxis(Hand.kRight));

			SmartDashboard.putBoolean("drA", this.robot.driver.getAButton());

			// Wrist stuff
			SmartDashboard.putNumber("SetPoint", this.wristSetPoint);
			SmartDashboard.putNumber("Raw Wrist Position", this.robot.wrist.getPosition());
			SmartDashboard.putNumber("Adjusted Wrist Position (Ang)",
					wristTickToAng(this.robot.wrist.getPosition() - this.straightUp));
			neg = SmartDashboard.getBoolean("neg", this.neg);
			// Ball?
			SmartDashboard.putBoolean("Has ball", !this.robot.ballIn.get());

			// If the limelight stuff is not null, show its values
			if (RobotMap.limelight != null) {
				SmartDashboard.putNumber("Limelight Vertical Offset", RobotMap.limelight.getEntry("ty").getDouble(0));

				if (this.isDisabled()) {
					boolean LEDs = SmartDashboard.getBoolean("LED", true);
					if (LEDs) {
						this.robot.enableLimelightLEDs();
					} else {
						this.robot.disableLimelightLEDs();
					}
				}

				// d = (h2-h1) / tan(a1+a2)
				double distance = (28.5 - 11)
						/ Math.tan(Math.toRadians(this.robot.limelight.getEntry("tx").getDouble(0)));
			}

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
		this.teleopPeriodic();
	}

	/**
	 * Periodic code for teleop mode should go here.
	 */
	@Override
	public void teleopPeriodic() {
		try {

			if (this.neg) {
				zero = -2063;
			} else {
				zero = 2063;
			}
			this.minus180 = this.zero - 2048;
			this.straightUp = (this.zero + this.minus180) / 2;

			// Limelight stuff
			this.pid.setSetpoint(this.visSetpoint);
			if (this.wristSetPoint == zero || this.robot.limelight.getEntry("tv").getDouble(0) != 1) {
				this.visEnabled = false;
			}
			if (this.visEnabled) {
				this.vis = this.pid.getOutput(this.robot.limelight.getEntry("ty").getDouble(0));
			} else {
				this.vis = 0;
			}

			// Calculate drive power
			double forward = Math.pow(this.robot.driver.getY(Hand.kLeft), 1) * .8,
					turn = Math.pow(this.robot.driver.getX(Hand.kRight), 1) * .6;
			if (Math.abs(forward) < 0.2d) {
				forward = 0;
			}
			if (Math.abs(turn) < 0.2d) {
				turn = 0;
			}

			// West coast drive with PID
			if (this.robot.driver.getBumper(Hand.kLeft)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * this.maxVelDT - this.vis);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * this.maxVelDT + this.vis);
			} else if (this.robot.driver.getBumper(Hand.kRight)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn * .5) * .4 * this.maxVelDT - this.vis);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn * .5) * .4 * this.maxVelDT + this.vis);
			} else {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * .4 * this.maxVelDT - this.vis);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * .4 * this.maxVelDT + this.vis);
			}
			if (forward == 0 && turn == 0) {
				this.robot.leftDrive.set(ControlMode.Velocity, -this.vis);
				this.robot.rightDrive.set(ControlMode.Velocity, +this.vis);
			}

			// Controller vibration
			if (Math.abs(this.robot.limelight.getEntry("ty").getDouble(0) - this.visSetpoint) < 1 && this.visEnabled) {
				this.robot.driver.setRumble(RumbleType.kLeftRumble, .1);
				this.robot.driver.setRumble(RumbleType.kRightRumble, .1);

				// I made tristan's controller more intense becasye ge likes it ;)
				this.robot.operator.setRumble(RumbleType.kLeftRumble, .3);
				this.robot.operator.setRumble(RumbleType.kRightRumble, .3);
			} else {
				this.robot.driver.setRumble(RumbleType.kLeftRumble, 0);
				this.robot.driver.setRumble(RumbleType.kRightRumble, 0);
				this.robot.operator.setRumble(RumbleType.kLeftRumble, 0);
				this.robot.operator.setRumble(RumbleType.kRightRumble, 0);
			}

			// Vision?
			if (this.robot.driver.getAButtonPressed()) {
				this.visEnabled = !this.visEnabled;
			}
			SmartDashboard.putBoolean("Vision Enabled", this.visEnabled);
			SmartDashboard.putNumber("Vision Output", this.vis);

			// Outtake stuff
			// If the sensor sees nothing
			if (this.robot.ballIn.get()) {
				if (this.robot.operator.getTriggerAxis(Hand.kLeft) > .3) {
					this.robot.leftOuttake.setPower(this.outtakePresetSpeed);
				} else if (this.robot.operator.getTriggerAxis(Hand.kRight) > .3) {
					this.robot.leftOuttake.setPower(-this.outtakePresetSpeed);
				}
				// if no input on op, set to 0
				else {
					this.robot.leftOuttake.setPower(0);
				}
			}
			// if a ball is sensed only allow outtaking
			else {
				if (this.robot.operator.getTriggerAxis(Hand.kRight) > .3) {
					this.robot.leftOuttake.setPower(-this.outtakePresetSpeed);
				}
				// Stop
				else {
					this.robot.leftOuttake.setPower(0);
				}
			}

			// Wrist
			// Update desired position
			if (Math.abs(this.robot.operator.getY(Hand.kLeft)) > .2d) {
				this.wristSetPoint += this.robot.operator.getY(Hand.kLeft) * 40;
			}
			// Gets d-pad inputs
			if (this.robot.operator.getPOV() == 0) { // straight up
				this.wristSetPoint = this.straightUp;
			} else if (this.robot.operator.getPOV() == 90) { // minus 180
				this.wristSetPoint = this.straightUp - 1024;
			} else if (this.robot.operator.getPOV() == 270) { // zero
				this.wristSetPoint = this.straightUp + 1024;
			} else if (this.robot.operator.getBumper(Hand.kRight)) {
				this.wristSetPoint = this.straightUp - this.exchangePosOffset;
			} else if (this.robot.operator.getBumper(Hand.kLeft)) {
				this.wristSetPoint = this.straightUp + this.exchangePosOffset;
			}

			// arbitrary feed forward accounts for gravity
			this.arbFeedForward = -Math
					.sin(wristTickToAng(this.robot.wrist.getSelectedSensorPosition() - this.straightUp) * (Math.PI / 180))
					* this.kG;
			if (!this.manualOveride) {
				this.robot.wrist.set(ControlMode.MotionMagic, this.wristSetPoint, DemandType.ArbitraryFeedForward,
						this.arbFeedForward);
			} else {
				this.robot.wrist.set(ControlMode.PercentOutput, this.robot.operator.getY(Hand.kLeft));
			}
			if (this.robot.operator.getBButtonPressed()) {
				this.manualOveride = !manualOveride;
			}

			// Hatch mechanism
			if (this.robot.operator.getAButtonPressed()) {
				this.robot.toggleHatchMechanism();
			}
			if (this.robot.operator.getBButton()) {

			}
			if (this.robot.operator.getXButtonPressed()) {
				this.robot.toggleHatchExtension();
			}

			if (this.robot.limelight != null) {
				if (this.wristSetPoint == this.zero) {
					this.robot.disableLimelightLEDs();
				} else {
					this.robot.enableLimelightLEDs();
				}
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