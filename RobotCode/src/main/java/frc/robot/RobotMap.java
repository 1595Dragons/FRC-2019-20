package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

	// Practice Bot? - TODO: double check
	/*
	 * private final int leftDrive1Port = 9, leftDrive2Port = 7, leftDrive3Port = 0,
	 * rightDrive1Port = 14, rightDrive2Port = 12, rightDrive3Port = 2;
	 */

	// Real Robot
	private final int leftDrive1Port = 5, leftDrive2Port = 6, leftDrive3Port = 7, rightDrive1Port = 8,
			rightDrive2Port = 10, rightDrive3Port = 11, wristPort = 9, leftIntakePort = 0, rightIntakePort = 12;

	// Get the Solenoid ports off of the PCM
	private final int popperPort = -1, extenderPort = -1, clamperPort = -1; // TODO: Find correct ports
	/**
	 * Declare the motors that will be used on the robot and will be used by other
	 * classes. Don't initalize them yet.
	 * 
	 * (Wait untill the constructor to do that).
	 * 
	 */
	public Motor leftDrive, rightDrive, wrist, leftIntake, rightIntake;

	/**
	 * Declare the solenoids that will be used in the robot, but keep them private,
	 * in order to encourage the use of the functions in this class. Also dont
	 * initalize them.
	 */
	private Solenoid popper, extender, clamper;

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
	private boolean hatchPanelSecured = false, hatchMechDeployed = false, popped = false;

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
		SmartDashboard.putNumber("D-pad", driver.getPOV());

		// Apply port addresses to the robot
		this.leftDrive = new Motor(this.leftDrive1Port);
		this.leftDrive2 = new Motor(this.leftDrive2Port);
		this.leftDrive3 = new Motor(this.leftDrive3Port);
		this.rightDrive = new Motor(this.rightDrive1Port);
		this.rightDrive2 = new Motor(this.rightDrive2Port);
		this.rightDrive3 = new Motor(this.rightDrive3Port);
		this.wrist = new Motor(this.wristPort);
		this.leftIntake = new Motor(this.leftIntakePort);
		this.rightIntake = new Motor(this.rightIntakePort);

		// Setup the encoders
		this.popper = new Solenoid(popperPort);
		this.popper.setPulseDuration(0.02d);
		this.extender = new Solenoid(extenderPort);
		this.extender.setPulseDuration(0.02d);
		this.clamper = new Solenoid(clamperPort);
		this.clamper.setPulseDuration(0.02d);

		// Setup encoders
		// this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		// this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		// Set the secondary motors to follow the first ones
		this.leftDrive2.set(ControlMode.Follower, this.leftDrive1Port);
		this.leftDrive3.set(ControlMode.Follower, this.leftDrive1Port);
		this.rightDrive2.set(ControlMode.Follower, this.rightDrive1Port);
		this.rightDrive3.set(ControlMode.Follower, this.rightDrive1Port);
		this.rightIntake.set(ControlMode.Follower, this.leftIntakePort);

		// Invert necessary drive motors
		this.leftDrive.setInverted(true);
		this.leftDrive2.setInverted(true);
		this.leftDrive3.setInverted(true);
		this.rightIntake.setInverted(true);

		// State whether the sensor is in phase with the motor
		this.wrist.setSensorPhase(true);

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
		this.clamper.set(false);
		this.hatchPanelSecured = false;
	}

	/**
	 * Secures the hatch panel. (Closes the hatch mechanism).
	 */
	public void secureHatchPanel() {
		this.clamper.set(true);
		this.hatchPanelSecured = true;
	}

	/**
	 * Deploys the hatch mech.
	 */
	public void extendHatch() {
		this.extender.set(true);
		this.hatchMechDeployed = true;
	}

	/**
	 * Retracts the hatch mech.
	 */
	public void retracthHatch() {
		this.extender.set(false);
		this.hatchMechDeployed = false;
	}

	/**
	 * Pops the ball.
	 */
	public void pop() {
		this.popper.set(true);
		this.popped = true;
	}

	/**
	 * Petracts the popper.
	 */
	public void dePop() {
		this.popper.set(false);
		this.popped = false;
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
	 * Toggles the hatch extension mechanism.
	 */
	public void toggleHatchExtension() {
		if (this.hatchMechDeployed) {
			this.retracthHatch();
		} else {
			this.extendHatch();
		}
	}

	/**
	 * Toggles the ball popper doo-dad.
	 */
	public void togglePopper() {
		if (this.popped) {
			this.dePop();
		} else {
			this.pop();
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