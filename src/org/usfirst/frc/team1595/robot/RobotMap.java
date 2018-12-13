/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1595.robot;

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
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

	// For this example code, Ill be going off of the Chicago robot

	// First, start with the ports
	private final int leftDrive1Port = 8, leftDrive2Port = 11, rightDrive1Port = 7, rightDrive2Port = 14,
			leftIntakePort = 3, rightIntakePort = 2, armPort = 9, gamepad1Port = 0, gamepad2Port = 1;

	// Now, setup the motor objects
	TalonSRX leftDrive1 = new TalonSRX(leftDrive1Port), leftDrive2 = new TalonSRX(leftDrive2Port),
			rightDrive1 = new TalonSRX(rightDrive1Port), rightDrive2 = new TalonSRX(rightDrive2Port),
			leftIntake = new TalonSRX(leftIntakePort), rightIntake = new TalonSRX(rightIntakePort),
			arm = new TalonSRX(armPort);

	// Also setup the controllers
	XboxController gamepad1 = new XboxController(gamepad1Port), gamepad2 = new XboxController(gamepad2Port);
	
	
	// Init the robot map
	RobotMap() {
		
		// Set the secondary drive motors to follow the OG motor
		leftDrive2.set(ControlMode.Follower, leftDrive1Port);
		rightDrive2.set(ControlMode.Follower, rightDrive1Port);
		rightIntake.set(ControlMode.Follower, leftIntakePort);
		leftDrive1.setInverted(true);
		
		
	
	}
	

}
