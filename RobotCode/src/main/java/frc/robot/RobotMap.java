package frc.robot;
/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// First, start with the ports for the Chicago robot
	private final int leftDrive1Port = 8, leftDrive2Port = 11, rightDrive1Port = 7, rightDrive2Port = 14,
			solenoidOnPort = 0, solenoidOffPort = 1;

	// Now, delcare the drive motors that are on the robot
	public TalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2;

	// Also setup the controllers for the drivers
	public XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

	// Setup the pnumatics
	Solenoid hatchRelease, hatchClamp;

	// Is vision supported in driver station?
	// (Default is true)
	public boolean isVisionSupported = true;

	// Init the robot map
	RobotMap() {

		// Apply port addresses to the robot
		this.leftDrive1 = new TalonSRX(this.leftDrive1Port);
		this.leftDrive2 = new TalonSRX(this.leftDrive2Port);
		this.rightDrive1 = new TalonSRX(this.rightDrive1Port);
		this.rightDrive2 = new TalonSRX(this.rightDrive2Port);

		// Setup encoders
		this.leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);


		// Set the secondary motors to follow the first ones
		this.leftDrive2.set(ControlMode.Follower, this.leftDrive1Port);
		this.rightDrive2.set(ControlMode.Follower, this.rightDrive1Port);

		// Invert necessary drive motors
		this.leftDrive1.setInverted(true);

		// Setup solenoids
		this.hatchClamp = new Solenoid(this.solenoidOnPort);
		this.hatchRelease = new Solenoid(this.solenoidOffPort);
		this.hatchClamp.setPulseDuration(0.5d);
		this.hatchRelease.setPulseDuration(0.5d);
	}

	public void releaseHatchPanel() {
		this.hatchRelease.startPulse();;
	}

	public void secureHatchPanel() {
		this.hatchClamp.startPulse();
	}
}