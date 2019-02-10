package frc.robot;
/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	/**
	 * Get the port mappings for the robot
	 * TODO
	 */
	private final int leftDrive1Port = 0, leftDrive2Port = 1, leftDrive3Port = 2, rightDrive1Port = 3, rightDrive2Port = 4, rightDrive3Port = 5;

	/**
	 * Decalre the motrors used on the robot, but dont initalize them yet.
	 * 
	 * (Wait untill the constructor to do that)
	 */
	public Motor leftDrive, rightDrive;
	private Motor leftDrive2, rightDrive2, leftDrive3, rightDrive3;

	/**
	 * Setup the controllers for the drivers. No need to wait for the constructor on
	 * this one.
	 */
	public XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

	/**
	 * Create the object for the driver camera, as well as the vision camera (if one
	 * is mounted, then set it up in the constructor).
	 * 
	 * Also, the stream produced by the camera can be viewed at:
	 * {@link https://roborio-1595-frc.local:1181/?action=stream}
	 */
	public UsbCamera driverCam;

	private boolean hatchPanelSecured = false;

	/**
	 * Setup everything on the robot.
	 * 
	 * Be sure to run this once (Preferable either in the start of a class, or
	 * during <code>robotInit()</code>).
	 */
	RobotMap() {

		// Apply port addresses to the robot
		this.leftDrive = new Motor(this.leftDrive1Port);
		this.leftDrive2 = new Motor(this.leftDrive2Port);
		this.leftDrive3 = new Motor(this.leftDrive3Port);
		this.rightDrive = new Motor(this.rightDrive1Port);
		this.rightDrive2 = new Motor(this.rightDrive2Port);
		this.rightDrive3 = new Motor(this.rightDrive3Port);

		// Setup encoders
		this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		// Set the secondary motors to follow the first ones
		this.leftDrive2.set(ControlMode.Follower, this.leftDrive1Port);
		this.leftDrive3.set(ControlMode.Follower, this.leftDrive1Port);
		this.rightDrive2.set(ControlMode.Follower, this.rightDrive1Port);
		this.rightDrive3.set(ControlMode.Follower, this.rightDrive1Port);

		// Invert necessary drive motors
		// TODO

		// Setup camera (this has a high liklyhood of breaking, so surround it with a
		// try catch block)
		try {
			this.driverCam = CameraServer.getInstance().startAutomaticCapture(0);
			this.driverCam.setFPS(15);
			this.driverCam.setResolution(320, 240);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Releases the hatch panel. (Opens the hatch mechanism).
	 */
	public void releaseHatchPanel() {
		// TODO
		this.hatchPanelSecured = true;
	}

	/**
	 * Secures the hatch panel. (Closes the hatch mechanism).
	 */
	public void secureHatchPanel() {
		// TODO
		this.hatchPanelSecured = true;
	}

	/**
	 * Toggles the hatch panel mechanism. Either releasing or securing the hatch
	 * panel.
	 */
	public void toggleHatchMechanism() {
		if (this.hatchPanelSecured) {
			this.releaseHatchPanel();
		} else {
			this.secureHatchPanel();
		}
	}
}