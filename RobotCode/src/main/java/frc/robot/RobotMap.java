package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
	 * Create a boolean to check if this is the practice robot
	 */
	public boolean PRACTICEBOT = false;

	/**
	 * Practice robot ports
	 */
	private final int PracticeleftDrive1Port = 20, PracticeleftDrive2Port = 19, PracticeleftDrive3Port = 17,
			PracticerightDrive1Port = 24, PracticerightDrive2Port = 23, PracticerightDrive3Port = 21,
			PracticewristPort = 22, PracticeleftOuttakePort = 13, PracticerightOuttakePort = 18;

	/**
	 * Real robot
	 */
	private final int leftDrive1Port = 5, leftDrive2Port = 6, leftDrive3Port = 7, rightDrive1Port = 10,
			rightDrive2Port = 8, rightDrive3Port = 11, wristPort = 9, leftOuttakePort = 0, rightOuttakePort = 12;

	// Get the Solenoid ports off of the PCM

	// Practice Bot
	private final int PracticeextenderPort1 = 1,PracticeextenderPort2 = 4, PracticeclamperPort1 = 0, PracticeclamperPort2 = 5;
	private final int Practicenothing1Port = 2, Practicenothing2Port = 7;


	// Real Robot
	private final int extenderPort1 = 2, extenderPort2 = 6, clamperPort1 = 3, clamperPort2 = 7; // TODO: Find correct ports
	private final int nothing1Port = 4, nothing2Port = 0;

	private final int ballInPort = 0, limeLightServoPort = 1;
	
	private int currentlimit = 10;

	/**
	 * Declare the motors that will be used on the robot and will be used by other
	 * classes. Don't initalize them yet.
	 * 
	 * (Wait untill the constructor to do that).
	 * 
	 */
	public Motor leftDrive, rightDrive, wrist, leftOuttake, rightOuttake;

	/**
	 * Declare the solenoids that will be used in the robot, but keep them private,
	 * in order to encourage the use of the functions in this class. Also dont
	 * initalize them.
	 */
	private DoubleSolenoid extender, clamper;
	public Solenoid nothing1, nothing2;

	/**
	 * Declare the motors that will be used on the robot, but that shouldnt be used
	 * by other classes. This is most commonly the slave motors used on the drive
	 * train.
	 */
	private Motor leftDrive2, rightDrive2, leftDrive3, rightDrive3;

	public DigitalInput ballIn;
	public Servo limeLightServo;
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
	public edu.wpi.cscore.UsbCamera driverCam1, driverCam2;

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

		// Apply port addresses to the robot, based on whether or not it is the practice
		// bot.
		if (this.PRACTICEBOT) {
			this.leftDrive = new Motor(this.PracticeleftDrive1Port);
			this.leftDrive2 = new Motor(this.PracticeleftDrive2Port);
			this.leftDrive3 = new Motor(this.PracticeleftDrive3Port);
			this.rightDrive = new Motor(this.PracticerightDrive1Port);
			this.rightDrive2 = new Motor(this.PracticerightDrive2Port);
			this.rightDrive3 = new Motor(this.PracticerightDrive3Port);
			this.wrist = new Motor(this.PracticewristPort);
			this.leftOuttake = new Motor(this.PracticeleftOuttakePort);
			this.rightOuttake = new Motor(this.PracticerightOuttakePort);
		} else {
			this.leftDrive = new Motor(this.leftDrive1Port);
			this.leftDrive2 = new Motor(this.leftDrive2Port);
			this.leftDrive3 = new Motor(this.leftDrive3Port);
			this.rightDrive = new Motor(this.rightDrive1Port);
			this.rightDrive2 = new Motor(this.rightDrive2Port);
			this.rightDrive3 = new Motor(this.rightDrive3Port);
			this.wrist = new Motor(this.wristPort);
			this.leftOuttake = new Motor(this.leftOuttakePort);
			this.rightOuttake = new Motor(this.rightOuttakePort);
		}

		if (this.PRACTICEBOT == false) {
			this.extender = new DoubleSolenoid(extenderPort1, extenderPort2);
			this.clamper = new DoubleSolenoid(clamperPort1, clamperPort2);
			this.nothing1 = new Solenoid(nothing1Port);
			this.nothing2 = new Solenoid(nothing2Port);
		} else {
			this.extender = new DoubleSolenoid(PracticeextenderPort1, PracticeextenderPort2);
			this.clamper = new DoubleSolenoid(PracticeclamperPort1, PracticeclamperPort2);
			this.nothing1 = new Solenoid(Practicenothing1Port);
			this.nothing2 = new Solenoid(Practicenothing2Port);
		}
		// Setup encoders
		this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		this.wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		this.ballIn = new DigitalInput(ballInPort);
		this.limeLightServo = new Servo(limeLightServoPort);

		// Set the secondary motors to follow the first ones
		this.leftDrive2.set(ControlMode.Follower, this.leftDrive.getDeviceID());
		this.leftDrive3.set(ControlMode.Follower, this.leftDrive.getDeviceID());
		this.rightDrive2.set(ControlMode.Follower, this.rightDrive.getDeviceID());
		this.rightDrive3.set(ControlMode.Follower, this.rightDrive.getDeviceID());
		this.rightOuttake.set(ControlMode.Follower, this.leftOuttake.getDeviceID());

		// Set the motors to break
		this.rightOuttake.setNeutralMode(NeutralMode.Brake);
		this.leftOuttake.setNeutralMode(NeutralMode.Brake);
		this.rightDrive.setNeutralMode(NeutralMode.Brake);
		this.leftDrive.setNeutralMode(NeutralMode.Brake);

		// Invert necessary drive motors
		this.leftDrive.setInverted(true);
		this.leftDrive2.setInverted(true);
		this.leftDrive3.setInverted(true);
		this.rightOuttake.setInverted(true);

		// State whether the sensor is in phase with the motor
		this.rightDrive.setSensorPhase(true);
		this.leftDrive.setSensorPhase(true);
		this.wrist.setSensorPhase(true);

		// Config current limit
		this.leftDrive.configContinuousCurrentLimit(currentlimit, 25);
		this.rightDrive.configContinuousCurrentLimit(currentlimit, 25);
		this.wrist.configContinuousCurrentLimit(currentlimit, 25);

		// Setup camera (this has a high liklyhood of breaking, so surround it with a
		// try catch block)
		try {
			this.driverCam1 = CameraServer.getInstance().startAutomaticCapture(0);
			this.driverCam1.setFPS(15);
			this.driverCam1.setResolution(320, 240);
			this.driverCam2 = CameraServer.getInstance().startAutomaticCapture(1);
			this.driverCam2.setFPS(15);
			this.driverCam2.setResolution(320, 240);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Releases the hatch panel. (Opens the hatch mechanism).
	 */
	public void releaseHatchPanel() {
		this.clamper.set(Value.kReverse);
		this.hatchPanelSecured = false;
	}

	/**
	 * Secures the hatch panel. (Closes the hatch mechanism).
	 */
	public void secureHatchPanel() {
		this.clamper.set(Value.kForward);
		this.hatchPanelSecured = true;
	}

	/**
	 * Deploys the hatch mech.
	 */
	public void extendHatch() {
		this.extender.set(Value.kForward);
		this.hatchMechDeployed = true;
	}

	/**
	 * Retracts the hatch mech.
	 */
	public void retracthHatch() {
		this.extender.set(Value.kReverse);
		this.hatchMechDeployed = false;
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
		if (Math.abs(power) > 0.2d) {
			this.chooser.getSelected().setPower(power);
		} else {
			this.chooser.getSelected().stop();
		}
	}

}