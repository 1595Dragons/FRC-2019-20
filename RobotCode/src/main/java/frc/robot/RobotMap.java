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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	/**
	 * Get the port mappings for the China robot
	 */
	private final int leftDrive1Port = 8, leftDrive2Port = 11, rightDrive1Port = 7, rightDrive2Port = 14,
			solenoidOnPort = 0, solenoidOffPort = 1;

	/**
	 * Decalre the motrors used on the robot, but dont initalize them yet.
	 * 
	 * (Wait untill the constructor to do that)
	 */
	public TalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2, lift1, lift2, lift3, leftIntake, rightIntake;

	/**
	 * Setup the controllers for the drivers. No need to wait for the constructor on
	 * this one.
	 */
	public XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

	/**
	 * Setup the pnumatics, but use public methods to manage how these work.
	 */
	private Solenoid hatchRelease, hatchClamp;

	/**
	 * Setup a variable for the hatch mechanism to check if the hatch panel is
	 * secured.
	 * 
	 * (Default is false)
	 */
	public boolean hatchPanelSecured = false;

	/**
	 * Create the object for the driver camera, as well as the vision camera (if one
	 * is mounted, then set it up in the constructor).
	 * 
	 * Also, the stream produced by the camera can be viewed at:
	 * {@link https://roborio-1595-frc.local:1181/?action=stream}
	 */
	public UsbCamera driverCam, visionCam;

	/**
	 * Vision object for running GRIP pipeline, in order to track the vision targets.
	 * In the future, we will idealy have a limelight do this for us.
	 */
	public Vision vision = new Vision();

	/**
	 * Setup all the motors and solendoids in the robot.
	 * 
	 * Be sure to run this once (Preferable either in the start of a class, or
	 * during <code>robotInit()</code>).
	 */
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
		this.hatchClamp.setPulseDuration(0.02d);
		this.hatchRelease.setPulseDuration(0.02d);

		// Setup camera (this has a high liklyhood of breaking, so surround it with a
		// try catch block)
		try {
			this.driverCam = CameraServer.getInstance().startAutomaticCapture(0);
			this.driverCam.setFPS(15);
			this.driverCam.setResolution(320, 240);

			// If there is a vision camera mounted, uncomment the following block of code
			/*
			 * this.visionCam = CameraServer.getInstance().startAutomaticCapture(1);
			 * this.visionCam.setBrightness(0); this.visionCam.setWhiteBalanceManual(10000);
			 * this.visionCam.setResolution(320, 240); this.visionCam.setExposureManual(0);
			 * 
			 * // This is the part that has the highest chance of throwing an error
			 * vision.generateTargetImage(this.visionCam);
			 */
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Releases the hatch panel. (Opens the hatch mechanism).
	 */
	public void releaseHatchPanel() {
		this.hatchRelease.startPulse();
		this.hatchPanelSecured = false;
	}

	/**
	 * Secures the hatch panel. (Closes the hatch mechanism).
	 */
	public void secureHatchPanel() {
		this.hatchClamp.startPulse();
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