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
	private final int chicagoLeftDrive1Port = 8, chicagoLeftDrive2Port = 11, chicagoRightDrive1Port = 7,
			chicagoRightDrive2Port = 14, chicagoLeftIntakePort = 3, chicagoRightIntakePort = 2, flippyBoiPort = 9;

	// Now, setup the motor objects that are used on both robots
	TalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2, leftIntake, rightIntake;

	// Setup motors that are used ONLY on the Chicago robot
	TalonSRX flippyBoi;

	// Also setup the controllers
	XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

	// Init the robot map
	RobotMap(RobotType type) {

		// Apply port addresses based on robot

		// Setup the motors to go off of the Chicago robot's ports
		leftDrive1 = new TalonSRX(chicagoLeftDrive1Port);
		leftDrive2 = new TalonSRX(chicagoLeftDrive2Port);
		rightDrive1 = new TalonSRX(chicagoRightDrive1Port);
		rightDrive2 = new TalonSRX(chicagoRightDrive2Port);
		leftIntake = new TalonSRX(chicagoLeftIntakePort);
		rightIntake = new TalonSRX(chicagoRightIntakePort);
		flippyBoi = new TalonSRX(flippyBoiPort);

		// Set the secondary drive motors to follow the OG motor
		leftDrive2.set(ControlMode.Follower, chicagoLeftDrive1Port);
		rightDrive2.set(ControlMode.Follower, chicagoRightDrive1Port);
		leftDrive1.setInverted(true);

	}
}