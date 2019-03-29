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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private boolean neg = true, manualOveride = false, m_LimelightHasValidTarget = false, manualWrist = false, visEnabled;
	private double kP = 2, kI = 0.001, kD = 0, kF = 0, kG = 0.075, DTkP = 5,
			DTkI = 0.003, DTkD = 0, LDTkF = 1, RDTkF = 1, maxVelDT = 400, arbFeedForward = 0,
			wristSetPoint, outtakePresetSpeed = .55, wristTicksPerDeg = 2048 / 180, exchangePosOffset = 300, zero,
			minus180, straightUp, m_LimelightDriveCommand = 0, m_LimelightSteerCommand = 0;

	double STEER_K = 6, DRIVE_K = 0, DESIRED_TARGET_AREA = 7.1, MAX_DRIVE = 100;

	int counter = 0;
	
	private int iZone = 100, cruiseVel = 200, maxAccel = 800, forwardLimit = 90, backwardLimit = 90, kTimeOutMs = 25;

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

		// Setup individual drive motor tests
		this.robot.setupTestMode();

		// PID for wrist
		SmartDashboard.putNumber("STEER_K", this.STEER_K);
		SmartDashboard.putNumber("DRIVE_K", this.DRIVE_K);
		SmartDashboard.putNumber("DESIRED_TARGET_AREA", this.DESIRED_TARGET_AREA);
		SmartDashboard.putNumber("MAX_DRIVE", this.MAX_DRIVE);
		SmartDashboard.putNumber("kP", this.kP);
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
			this.zero = 2162;//2241;
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

		// PIDF && Motion Profile constants / configs for the wrist
		this.kP = SmartDashboard.getNumber("kP", this.kP);
		this.kI = SmartDashboard.getNumber("kI", this.kI);
		this.kD = SmartDashboard.getNumber("kD", this.kD);
		this.kF = SmartDashboard.getNumber("kF", this.kF);
		this.kG = SmartDashboard.getNumber("kG", this.kG);
		this.STEER_K = SmartDashboard.getNumber("STEER_K", this.STEER_K);
		this.DRIVE_K = SmartDashboard.getNumber("DRIVE_K", this.DRIVE_K);
		this.DESIRED_TARGET_AREA = SmartDashboard.getNumber("DESIRED_TARGET_AREA", this.DESIRED_TARGET_AREA);
		this.MAX_DRIVE = SmartDashboard.getNumber("MAX_DRIVE", this.MAX_DRIVE);
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


			SmartDashboard.putNumber("ty", robot.limelight.getEntry("ty").getDouble(0));

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
			if (this.robot.limelight != null) {
				SmartDashboard.putNumber("Limelight Vertical Offset", this.robot.limelight.getEntry("ty").getDouble(0));

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

			/*if (this.neg) {
				zero = -2063;
			} else {
				zero = 2063;
			}
			this.minus180 = this.zero - 2048;
			this.straightUp = (this.zero + this.minus180) / 2;*/

			// Calculate drive power
			double forward = Math.pow(this.robot.driver.getY(Hand.kLeft), 1) * .8,
					turn = Math.pow(this.robot.driver.getX(Hand.kRight), 1) * .6;
			if (Math.abs(forward) < 0.2d) {
				forward = 0;
			}
			if (Math.abs(turn) < 0.2d) {
				turn = 0;
			}
			Update_Limelight_Tracking();

			// West coast drive with PID
			double visLeft;
			double visRight;
			
			if(visEnabled){
				visLeft = -m_LimelightSteerCommand + m_LimelightDriveCommand;
				visRight = m_LimelightSteerCommand + m_LimelightDriveCommand;
			}
			else{
				visLeft = 0;
				visRight = 0;
			}
			if (this.robot.driver.getBumper(Hand.kLeft)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * this.maxVelDT + visLeft);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * this.maxVelDT + visRight);
			} else if (this.robot.driver.getBumper(Hand.kRight)) {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn * .5) * .4 * this.maxVelDT + visLeft);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn * .5) * .4 * this.maxVelDT + visRight);
			} else {
				this.robot.leftDrive.set(ControlMode.Velocity, (forward - turn) * .4 * this.maxVelDT + visLeft);
				this.robot.rightDrive.set(ControlMode.Velocity, (forward + turn) * .4 * this.maxVelDT + visRight);
			}
			if (forward == 0 && turn == 0) {
				this.robot.leftDrive.set(ControlMode.Velocity, + visLeft);
				this.robot.rightDrive.set(ControlMode.Velocity, + visRight);
			}

			// Controller vibration
			/*if (Math.abs(this.robot.limelight.getEntry("ty").getDouble(0) - this.visSetpoint) < 1 && this.visEnabled) {
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
			}*/

			// Vision?
			if (this.robot.driver.getAButton()) {
				this.visEnabled = true;
			}
			else{
				this.visEnabled = false;
			}
			SmartDashboard.putBoolean("Vision Enabled", this.visEnabled);

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
			this.arbFeedForward = -Math.sin(
					wristTickToAng(this.robot.wrist.getSelectedSensorPosition() - this.straightUp) * (Math.PI / 180))
					* this.kG;
				//if(!this.manualOveride){
				this.robot.wrist.set(ControlMode.MotionMagic, this.wristSetPoint, DemandType.ArbitraryFeedForward,
					this.arbFeedForward);
				/*}
				else{
					this.robot.wrist.set(ControlMode.PercentOutput, this.robot.operator.getY(Hand.kLeft);
				}
				if(this.robot.operator.getBButtonPressed()){
					this.manualOveride = !manualOveride;
				}*/

			// Hatch mechanism
			if (this.robot.operator.getAButtonPressed()) {
				this.robot.toggleHatchMechanism();
			}
			if(counter % 100 == 0){
				robot.wrist.setSelectedSensorPosition((int) this.convert0To4096(this.robot.wrist.getSelectedSensorPosition()));
			}
			if (this.robot.operator.getXButtonPressed()) {
				this.robot.toggleHatchExtension();
			}

			counter ++;

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void Update_Limelight_Tracking()
	{
		  // These numbers must be tuned for your Robot!  Be careful!
		  // how hard to turn toward the target
  		  // how hard to drive fwd toward the target
		  // Area of the target when the robot reaches the wall
		  // Simple speed limit so we don't drive too fast
	
		  double tv = robot.limelight.getEntry("tv").getDouble(0);
		  double tx = robot.limelight.getEntry("tx").getDouble(0);
		  double ty = robot.limelight.getEntry("ty").getDouble(0);
		  double ta = robot.limelight.getEntry("ta").getDouble(0);
  
		  if (tv < 1.0)
		  {
			m_LimelightHasValidTarget = false;
			m_LimelightDriveCommand = 0.0;
			m_LimelightSteerCommand = 0.0;
			return;
		  }
  
		  m_LimelightHasValidTarget = true;
  
		  // Start with proportional steering
		  double steer_cmd = tx * STEER_K;
		  m_LimelightSteerCommand = steer_cmd;
  
		  // try to drive forward until the target area reaches our desired area
		  double drive_cmd = (ty) * DRIVE_K;
  
		  // don't let the robot drive too fast into the goal
		  if (drive_cmd > MAX_DRIVE)
		  {
			drive_cmd = MAX_DRIVE;
		  }
		  m_LimelightDriveCommand = drive_cmd;
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

	public double convert0To4096(double x) {
		double a = x;
		while (a > 4096 || a < 0) {
			if (a > 4096) {
				a -= 4096;
			} else if (a < 0) {
				a += 4096;
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