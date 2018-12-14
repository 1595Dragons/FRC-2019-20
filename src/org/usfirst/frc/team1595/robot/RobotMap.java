package org.usfirst.frc.team1595.robot;

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
	private final int chicagoLeftDrive1Port = 8, chicagoLeftDrive2Port = 11, chicagoRightDrive1Port = 7, chicagoRightDrive2Port = 14,
			chicagoLeftIntakePort = 3, chicagoRightIntakePort = 2, flippyBoiPort = 9;

	
	// Then move on to defining ports for the China robot
	private final int chinaLeftDrive1Port = 3, chinaLeftDrive2Port = 2, chinaRightDrive1Port = 10, chinaRightDrive2Port = 9, 
			chinaLeftIntakePort = 8, chinaRightIntakePort = 7, liftDrive1Port = 5, liftDrive2Port = 6, liftDrive3Port = 4, wristPort = 11;

	
	// Now, setup the motor objects that are used on both robots
	TalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2, leftIntake, rightIntake;
	
	
	// Setup motors that are used ONLY on the Chicago robot
	TalonSRX flippyBoi;

	
	// Setup motors that are used ONLY on the China robot
	TalonSRX liftDrive1, liftDrive2, liftDrive3, wrist;

	
	// Also setup the controllers
	XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

	
	// Init the robot map
	RobotMap(RobotType type) {
		
		// Apply port addresses based on robot
		switch (type) {
		case Chicago_2018:

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
			break;
			
		case China_2018:

			// Setup the motors to go off of the China robot's ports
			leftDrive1 = new TalonSRX(chinaLeftDrive1Port);
			leftDrive2 = new TalonSRX(chinaLeftDrive2Port);
			rightDrive1 = new TalonSRX(chinaRightDrive1Port);
			rightDrive2 = new TalonSRX(chinaRightDrive2Port);
			leftIntake = new TalonSRX(chinaLeftIntakePort);
			rightIntake = new TalonSRX(chinaRightIntakePort);
			liftDrive1 = new TalonSRX(liftDrive1Port);
			liftDrive2 = new TalonSRX(liftDrive2Port);
			liftDrive3 = new TalonSRX(liftDrive3Port);
			wrist = new TalonSRX(wristPort);
			
			// Set the secondary drive motors to follow the OG motor
			leftDrive2.set(ControlMode.Follower, chinaLeftDrive1Port);
			rightDrive2.set(ControlMode.Follower, chinaRightDrive1Port);
			liftDrive2.set(ControlMode.Follower, liftDrive1Port);
			liftDrive3.set(ControlMode.Follower, liftDrive1Port);
			
			
			// Need to reverse the left side
			leftDrive1.setInverted(true);
			leftDrive2.setInverted(true);
			
			break;
		}
	}

}
