package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
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
	 * Create a boolean to check if this is the practice robot
	 */
	public boolean PRACTICEBOT = true;

	/**
	 * Network table used by the limelight
	 */
	public static NetworkTable limelight;

	/**
	 * Practice robot ports
	 */
	private static final int PracticeleftDrive1Port = 20, PracticeleftDrive2Port = 19, PracticeleftDrive3Port = 17,
			PracticerightDrive1Port = 24, PracticerightDrive2Port = 23, PracticerightDrive3Port = 21,
			PracticewristPort = 22, PracticeleftOuttakePort = 13, PracticerightOuttakePort = 18;

	/**
	 * Real robot
	 */
	private static final int leftDrive1Port = 5, leftDrive2Port = 6, leftDrive3Port = 7, rightDrive1Port = 10,
			rightDrive2Port = 8, rightDrive3Port = 11, wristPort = 9, leftOuttakePort = 0, rightOuttakePort = 12;

	/**
	 * Practice Bot solenoid ports
	 */
	private static final int PracticeextenderPort1 = 1, PracticeextenderPort2 = 4, PracticeclamperPort1 = 0,
			PracticeclamperPort2 = 5, Practicenothing1Port = 2, Practicenothing2Port = 7;

	/**
	 * Real Robot solenoid ports
	 */
	private static final int extenderPort1 = 2, extenderPort2 = 6, clamperPort1 = 3, clamperPort2 = 7, nothing1Port = 4,
			nothing2Port = 0, ballInPort = 0, limeLightServoPort = 1;

	private int currentlimit = 15;

	/**
	 * Declare the motors that will be used on the robot and will be used by other
	 * classes. Don't initalize them yet.
	 * 
	 * (Wait untill the constructor to do that).
	 * 
	 */
	public Motor leftDrive, rightDrive, leftOuttake, rightOuttake;
	public static TalonSRX wrist;

	/**
	 * Declare the solenoids that will be used in the robot, but keep them private,
	 * in order to encourage the use of the functions in this class. Also dont
	 * initalize them.
	 */
	public static DoubleSolenoid extender, clamper;
	public Solenoid nothing1, nothing2;

	/**
	 * Declare the motors that will be used on the robot, but that shouldnt be used
	 * by other classes. This is most commonly the slave motors used on the drive
	 * train.
	 */
	private Motor leftDrive2, rightDrive2, leftDrive3, rightDrive3;

	public static DigitalInput ballIn;

	/**
	 * Setup the controllers for the drivers.
	 */
	public final XboxController driver = new XboxController(0);
	@Deprecated
	public final XboxController operator = new XboxController(1);

	/**
	 * Create the object for the driver camera, as well as the vision camera (if one
	 * is mounted, then set it up in the constructor).
	 * 
	 * Also, the stream produced by the camera can be viewed at:
	 * {@link https://roborio-1595-frc.local:1181/?action=stream}
	 */
	public static edu.wpi.cscore.UsbCamera driverCam1, driverCam2;

	/**
	 * Declare a private global boolean for the hatch panel mechanism functions.
	 */
	@Deprecated
	public boolean hatchPanelSecured = false, hatchMechDeployed = false, popped = false;

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
			this.leftDrive = new Motor(RobotMap.PracticeleftDrive1Port);
			this.leftDrive2 = new Motor(RobotMap.PracticeleftDrive2Port);
			this.leftDrive3 = new Motor(RobotMap.PracticeleftDrive3Port);
			this.rightDrive = new Motor(RobotMap.PracticerightDrive1Port);
			this.rightDrive2 = new Motor(RobotMap.PracticerightDrive2Port);
			this.rightDrive3 = new Motor(RobotMap.PracticerightDrive3Port);
			RobotMap.wrist = new TalonSRX(RobotMap.PracticewristPort);
			this.leftOuttake = new Motor(RobotMap.PracticeleftOuttakePort);
			this.rightOuttake = new Motor(RobotMap.PracticerightOuttakePort);

			RobotMap.extender = new DoubleSolenoid(RobotMap.PracticeextenderPort1, RobotMap.PracticeextenderPort2);
			RobotMap.clamper = new DoubleSolenoid(RobotMap.PracticeclamperPort1, RobotMap.PracticeclamperPort2);
			this.nothing1 = new Solenoid(RobotMap.Practicenothing1Port);
			this.nothing2 = new Solenoid(RobotMap.Practicenothing2Port);
		} else {
			this.leftDrive = new Motor(RobotMap.leftDrive1Port);
			this.leftDrive2 = new Motor(RobotMap.leftDrive2Port);
			this.leftDrive3 = new Motor(RobotMap.leftDrive3Port);
			this.rightDrive = new Motor(RobotMap.rightDrive1Port);
			this.rightDrive2 = new Motor(RobotMap.rightDrive2Port);
			this.rightDrive3 = new Motor(RobotMap.rightDrive3Port);
			RobotMap.wrist = new TalonSRX(RobotMap.wristPort);
			this.leftOuttake = new Motor(RobotMap.leftOuttakePort);
			this.rightOuttake = new Motor(RobotMap.rightOuttakePort);

			RobotMap.extender = new DoubleSolenoid(RobotMap.extenderPort1, RobotMap.extenderPort2);
			RobotMap.clamper = new DoubleSolenoid(RobotMap.clamperPort1, RobotMap.clamperPort2);
			this.nothing1 = new Solenoid(RobotMap.nothing1Port);
			this.nothing2 = new Solenoid(RobotMap.nothing2Port);
		}

		// Setup limelight (only on comp robot)
		this.limelight = NetworkTableInstance.getDefault().getTable("limelight");

		// Setup encoders
		this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		this.wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		RobotMap.ballIn = new DigitalInput(ballInPort);

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
		RobotMap.wrist.setSensorPhase(true);

		// Config current limit
		this.leftDrive.configContinuousCurrentLimit(this.currentlimit);
		this.rightDrive.configContinuousCurrentLimit(this.currentlimit);
		this.leftDrive2.configContinuousCurrentLimit(this.currentlimit);
		this.leftDrive3.configContinuousCurrentLimit(this.currentlimit);
		this.rightDrive2.configContinuousCurrentLimit(this.currentlimit);
		this.rightDrive3.configContinuousCurrentLimit(this.currentlimit);
		RobotMap.wrist.configContinuousCurrentLimit(10);

		// Setup camera (this has a high liklyhood of breaking, so surround it with a
		// try catch block)
		try {
			RobotMap.driverCam1 = CameraServer.getInstance().startAutomaticCapture(0);
			RobotMap.driverCam1.setFPS(3);
			RobotMap.driverCam1.setResolution(320 / 2, 240 / 2);
			RobotMap.driverCam1.setExposureManual(40);
			RobotMap.driverCam1.setBrightness(40);
			RobotMap.driverCam2 = CameraServer.getInstance().startAutomaticCapture(1);
			RobotMap.driverCam2.setFPS(15);
			RobotMap.driverCam2.setResolution(320, 240);
			RobotMap.driverCam2.setExposureManual(75);
			RobotMap.driverCam2.setBrightness(75);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Releases the hatch panel. (Opens the hatch mechanism).
	 */
	@Deprecated
	public void releaseHatchPanel() {
		this.clamper.set(Value.kReverse);
		this.hatchPanelSecured = false;
	}

	/**
	 * Secures the hatch panel. (Closes the hatch mechanism).
	 */
	@Deprecated
	public void secureHatchPanel() {
		this.clamper.set(Value.kForward);
		this.hatchPanelSecured = true;
	}

	/**
	 * Deploys the hatch mech.
	 */
	@Deprecated
	public void extendHatch() {
		this.extender.set(Value.kForward);
		this.hatchMechDeployed = true;
	}

	/**
	 * Retracts the hatch mech.
	 */
	@Deprecated
	public void retracthHatch() {
		this.extender.set(Value.kReverse);
		this.hatchMechDeployed = false;
	}

	/**
	 * Toggles the hatch panel mechanism. Either releasing or securing the hatch
	 * panel.
	 */
	@Deprecated
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
	@Deprecated
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

	/**
	 * Turns on the limelight LEDs, blinding all in its path.
	 * 
	 * Mostly Tristan.
	 * 
	 * It just blinds Tristan.
	 */
	public void enableLimelightLEDs() {
		this.limelight.getEntry("ledMode").setNumber(3);
	}

	/**
	 * Turns iff the limelight LEDs.
	 */
	public void disableLimelightLEDs() {
		this.limelight.getEntry("ledMode").setNumber(1);
	}

}