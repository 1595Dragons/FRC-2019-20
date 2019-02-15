package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * 
 * @author Stephen - FRC 1595
 */
public class RobotMap {

	/**
	 * Get the port mappings for the robot.<br >
	 * <br >
	 * Use the phoenix tuner to find these values.
	 */
	private final int leftDrive1Port = 9, leftDrive2Port = 7, leftDrive3Port = 0, rightDrive1Port = 14,
			rightDrive2Port = 12, rightDrive3Port = 2;

	/**
	 * Declare the motors that will be used on the robot and will be used by other
	 * classes. Don't initalize them yet.
	 * 
	 * (Wait untill the constructor to do that).
	 * 
	 */
	public Motor leftDrive, rightDrive;

	/**
	 * Declare the motors that will be used on the robot, but that shouldnt be used
	 * by other classes. This is most commonly the slave motors used on the drive
	 * train.
	 */
	private Motor leftDrive2, rightDrive2, leftDrive3, rightDrive3;

	/**
	 * Setup the controllers for the drivers.
	 */
	public final XboxController driver = new XboxController(0), operator = new XboxController(1);

	/**
	 * Create the object for the driver camera, as well as the vision camera (if one
	 * is mounted, then set it up in the constructor).
	 * 
	 * Also, the stream produced by the camera can be viewed at:
	 * {@link https://roborio-1595-frc.local:1181/?action=stream}
	 */
	public edu.wpi.cscore.UsbCamera driverCam;

	/**
	 * Declare a private global boolean for the hatch panel mechanism functions.
	 */
	private boolean hatchPanelSecured = false;

	/**
	 * Declare a chooser (radio buttons on SmartDashboard) that will be used for
	 * test mode. The reason why this takes a <code>Motor</code> object is becasue
	 * the object the chooser will return is the individual motor to be run during
	 * test mode.
	 */
	private SendableChooser<Motor> chooser = new SendableChooser<>();

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
		// this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		// this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		// Set the secondary motors to follow the first ones
		this.leftDrive2.set(ControlMode.Follower, this.leftDrive1Port);
		this.leftDrive3.set(ControlMode.Follower, this.leftDrive1Port);
		this.rightDrive2.set(ControlMode.Follower, this.rightDrive1Port);
		this.rightDrive3.set(ControlMode.Follower, this.rightDrive1Port);

		// Invert necessary drive motors
		this.leftDrive.setInverted(true);
		this.leftDrive2.setInverted(true);
		this.leftDrive3.setInverted(true);

		// Setup camera (this has a high liklyhood of breaking, so surround it with a
		// try catch block)
		try {
			// this.driverCam = CameraServer.getInstance().startAutomaticCapture(0);
			// this.driverCam.setFPS(15);
			// this.driverCam.setResolution(320, 240);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Releases the hatch panel. (Opens the hatch mechanism).
	 */
	public void releaseHatchPanel() {
		// TODO add solenoids
		this.hatchPanelSecured = true;
	}

	/**
	 * Secures the hatch panel. (Closes the hatch mechanism).
	 */
	public void secureHatchPanel() {
		// TODO add solenoids
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

	/**
	 * This sets up the sendable chooser and its motors on the smart dashbaord for
	 * use in the test mode.
	 */
	public void setupTestMode() {
		// This is for running the motors one at a time during test mode
		this.chooser.setDefaultOption("Right 1", this.rightDrive);
		this.chooser.addOption("Right 2", this.rightDrive2);
		this.chooser.addOption("Right 3", this.rightDrive3);
		this.chooser.addOption("Left 1", this.leftDrive);
		this.chooser.addOption("Left 2", this.leftDrive2);
		this.chooser.addOption("Left 3", this.leftDrive3);

		// Add the chooser to smart dashboard
		edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(this.chooser);
	}

	/**
	 * When run, this will get the chosen motor from the chooser, and apply the
	 * necessary power to it. This is useful for testing individual motors, in order
	 * to check for things such as whether they are grinding against each other.
	 * <br >
	 * <br >
	 * Another way to check if the motors are grinding against each other is by
	 * checking the lights on the individual talons. If the light is green its going
	 * 'forward', if it's red it's going in 'reverse', and if it's orange its not
	 * moving.
	 */
	public void testMotors() {
		double power = this.driver.getY(edu.wpi.first.wpilibj.GenericHID.Hand.kLeft);
		if (Math.abs(power) > 0.1d) {
			this.chooser.getSelected().setPower(power);
		} else {
			this.chooser.getSelected().stop();
		}
	}

}