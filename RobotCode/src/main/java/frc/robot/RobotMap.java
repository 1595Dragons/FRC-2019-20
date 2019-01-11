package frc.robot;
/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// First, start with the ports for the Chicago robot
	private final int leftDrive1Port = 8, leftDrive2Port = 11, rightDrive1Port = 7, rightDrive2Port = 14;

	// Now, delcare the drive motors that are on the robot
	public TalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2;

	// Also setup the controllers for the drivers
	public XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

	// Init the robot map
	RobotMap() {

		// Apply port addresses to the robot
		leftDrive1 = new TalonSRX(leftDrive1Port);
		leftDrive2 = new TalonSRX(leftDrive2Port);
		rightDrive1 = new TalonSRX(rightDrive1Port);
		rightDrive2 = new TalonSRX(rightDrive2Port);

		// Set the secondary motors to follow the first ones
		leftDrive2.set(ControlMode.Follower, leftDrive1Port);
		rightDrive1.set(ControlMode.Follower, rightDrive1Port);

		// Invert necessary drive motors
		leftDrive1.setInverted(true);

	}
}