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
import frc.robot.commands.extender.toggleExtension;
import frc.robot.commands.mittens.toggleMitten;
import frc.robot.commands.wrist.MoveToUp;
import frc.robot.commands.wrist.WristPosition;
import frc.robot.controllers.operator;
import frc.robot.subsystems.Extenders;
import frc.robot.subsystems.Mittens;
import frc.robot.subsystems.Wrist;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private boolean neg = true, visEnabled = false;

	private double kP = 2, kI = 0.001, kD = 0, kF = 0, kG = 0.075, viskP = 4, viskI = 0.01, viskD = 0, DTkP = 5,
			DTkI = 0.003, DTkD = 0, LDTkF = 1, RDTkF = 1, maxVelDT = 400, arbFeedForward = 0, vis = 0, visSetpoint = 3,
			outtakePresetSpeed = .55, wristTicksPerDeg = 2048 / 180;

	private int iZone = 100, cruiseVel = 200, maxAccel = 800, forwardLimit = 90, backwardLimit = 90;

	private MiniPID pid = new MiniPID(viskP, viskI, viskD);

	private RobotMap robot = new RobotMap();

	// Subsystems for command based programming
	public static Extenders extender;
	public static Mittens mitten;
	public static Wrist wristSubsystem;
	public static operator op;

	@Override
	public void robotInit() {

		// Setup individual drive motor tests
		this.robot.setupTestMode();

		Robot.extender = new Extenders("Extender");
		Robot.mitten = new Mittens("Mitten");
		Robot.wristSubsystem = new Wrist("Wrist");

		try {
			Robot.op = new operator();
		} catch (Exception e) {
			System.err.println("Please plug in the operator controller");
		}

		// Add an option to smartdashboard to activate commands manually
		SmartDashboard.putData("Toggle mittens", new toggleMitten());
		SmartDashboard.putData("Toggle extension", new toggleExtension());
		SmartDashboard.putData("Wrist straight-up", new MoveToUp());

		SmartDashboard.putData(Scheduler.getInstance());

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
		RobotMap.wrist.configAllowableClosedloopError(0, 10);
		Wrist.setSetPoint(RobotMap.wrist.getSelectedSensorPosition());

		// Try to setup limelight
		if (RobotMap.limelight != null) {
			SmartDashboard.putBoolean("LED", true);
		}
	}

	@Override
	public void disabledInit() {
		Scheduler.getInstance().removeAll();
	}

	@Override
	public void autonomousInit() {
		this.teleopInit();

		// Close the hatch stuff
		Robot.mitten.secure();
	}

	@Override
	public void teleopInit() {
		this.robot.nothing1.set(true);
		this.robot.nothing2.set(true);

		// Limits for the wrist
		RobotMap.wrist.configForwardSoftLimitThreshold((int) (WristPosition.UP.getValue()
				+ wristAngToTick(this.forwardLimit) + (RobotMap.PRACTICEBOT ? 3937 : 0)));
		RobotMap.wrist.configReverseSoftLimitThreshold((int) (WristPosition.UP.getValue()
				- wristAngToTick(this.backwardLimit) + (RobotMap.PRACTICEBOT ? 3937 : 0)));
		RobotMap.wrist.configForwardSoftLimitEnable(true);
		RobotMap.wrist.configReverseSoftLimitEnable(true);

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
		RobotMap.wrist.config_kP(0, this.kP);
		RobotMap.wrist.config_kI(0, this.kI);
		RobotMap.wrist.config_kD(0, this.kD);
		RobotMap.wrist.config_kF(0, this.kF);
		RobotMap.wrist.config_IntegralZone(0, this.iZone);
		RobotMap.wrist.configMotionCruiseVelocity((int) (this.cruiseVel)); // ticks per 100MS
		RobotMap.wrist.configMotionAcceleration((int) (this.maxAccel)); // ticks per 100MS per second
		Wrist.setSetPoint(RobotMap.wrist.getSelectedSensorPosition());

		// Drive train PID
		this.DTkP = SmartDashboard.getNumber("DTkP", this.DTkP);
		this.DTkI = SmartDashboard.getNumber("DTkI", this.DTkI);
		this.DTkD = SmartDashboard.getNumber("DTkD", this.DTkD);
		this.LDTkF = SmartDashboard.getNumber("LDTkF", this.LDTkF);
		this.RDTkF = SmartDashboard.getNumber("RDTkF", this.RDTkF);
		RobotMap.leftDrive.config_kP(0, this.DTkP);
		RobotMap.leftDrive.config_kI(0, this.DTkI);
		RobotMap.leftDrive.config_kD(0, this.DTkD);
		RobotMap.leftDrive.config_kF(0, this.LDTkF);
		RobotMap.rightDrive.config_kP(0, this.DTkP);
		RobotMap.rightDrive.config_kI(0, this.DTkI);
		RobotMap.rightDrive.config_kD(0, this.DTkD);
		RobotMap.rightDrive.config_kF(0, this.RDTkF);

		// Outtake speed limit
		this.outtakePresetSpeed = SmartDashboard.getNumber("Outtake Preset Speed", this.outtakePresetSpeed);
	}

	@Override
	public void testInit() {
	}

	@Override
	public void robotPeriodic() {
		try {
			Scheduler.getInstance().run();

			// Drive train velocities
			SmartDashboard.putNumber("Left velocity", RobotMap.leftDrive.getSelectedSensorVelocity());
			SmartDashboard.putNumber("Right velocity", RobotMap.rightDrive.getSelectedSensorVelocity());

			SmartDashboard.putBoolean("Has ball", !RobotMap.ballIn.get());

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
						/ Math.tan(Math.toRadians(RobotMap.limelight.getEntry("tx").getDouble(0)));
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

	@Override
	public void autonomousPeriodic() {
		this.teleopPeriodic();
	}

	@Override
	public void teleopPeriodic() {
		try {
			// Limelight stuff
			this.pid.setSetpoint(this.visSetpoint);
			this.vis = this.visEnabled ? this.pid.getOutput(RobotMap.limelight.getEntry("ty").getDouble(0)) : 0;

			// Calculate drive power
			double forward = this.robot.driver.getY(Hand.kLeft) * .8, turn = this.robot.driver.getX(Hand.kRight) * .6;

			// Deadbands
			if (Math.abs(forward) < 0.2d) {
				forward = 0;
			}
			if (Math.abs(turn) < 0.2d) {
				turn = 0;
			}

			// West coast drive with PID
			if (this.robot.driver.getBumper(Hand.kLeft)) {
				RobotMap.leftDrive.set(ControlMode.Velocity, (forward - turn) * this.maxVelDT - this.vis);
				RobotMap.rightDrive.set(ControlMode.Velocity, (forward + turn) * this.maxVelDT + this.vis);
			} else if (this.robot.driver.getBumper(Hand.kRight)) {
				RobotMap.leftDrive.set(ControlMode.Velocity, (forward - turn * .5) * .4 * this.maxVelDT - this.vis);
				RobotMap.rightDrive.set(ControlMode.Velocity, (forward + turn * .5) * .4 * this.maxVelDT + this.vis);
			} else {
				RobotMap.leftDrive.set(ControlMode.Velocity, (forward - turn) * .4 * this.maxVelDT - this.vis);
				RobotMap.rightDrive.set(ControlMode.Velocity, (forward + turn) * .4 * this.maxVelDT + this.vis);
			}

			// Vision driving
			if (forward == 0 && turn == 0) {
				RobotMap.leftDrive.set(ControlMode.Velocity, -this.vis);
				RobotMap.rightDrive.set(ControlMode.Velocity, +this.vis);
			}

			// Controller vibration
			if (Math.abs(RobotMap.limelight.getEntry("ty").getDouble(0) - this.visSetpoint) < 1 && this.visEnabled) {
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
			// Get the speed for outtakeing
			double outspeed = this.robot.operator.getTriggerAxis(Hand.kRight) > .3 ? -this.outtakePresetSpeed : 0,
					inspeed = 0;

			// If the robot does not have the ball, allow intaking
			if (RobotMap.ballIn.get()) {
				inspeed = this.robot.operator.getTriggerAxis(Hand.kLeft) > .3 ? this.outtakePresetSpeed : 0;
			}

			// Apply that power
			RobotMap.outtake.set(ControlMode.PercentOutput, outspeed + inspeed);

			// Wrist
			// Update desired position
			if (Math.abs(this.robot.operator.getY(Hand.kLeft)) > .2d) {
				Wrist.setSetPoint(Wrist.getSetPoint() + (int) (this.robot.operator.getY(Hand.kLeft) * 40));
			}

			// arbitrary feed forward accounts for gravity
			this.arbFeedForward = -Math.sin(wristTickToAng(RobotMap.wrist.getSelectedSensorPosition()
					- (WristPosition.UP.getValue() + (RobotMap.PRACTICEBOT ? 3937 : 0))) * (Math.PI / 180)) * this.kG;
			if (!Wrist.manual) {
				RobotMap.wrist.set(ControlMode.MotionMagic, Wrist.getSetPoint(), DemandType.ArbitraryFeedForward,
						this.arbFeedForward);
			} else {
				RobotMap.wrist.set(ControlMode.PercentOutput, this.robot.operator.getY(Hand.kLeft));
			}

			if (this.robot.operator.getBButtonPressed()) {
				Wrist.manual = !Wrist.manual;
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

	@Override
	public void testPeriodic() {
		try {
			this.robot.testMotors();
		} catch (Exception error) {
			error.printStackTrace();
		}
	}

}