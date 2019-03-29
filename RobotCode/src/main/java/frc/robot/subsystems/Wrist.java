package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.wrist.WristPosition;

public class Wrist extends Subsystem {

	private static int wristSetpoint;

	public static boolean manual = false;

	public Wrist() {
		super("Wrist");
	}

	public Wrist(String name) {
		super(name);
	}

	@Override
	protected void initDefaultCommand() {
	}

	public void moveToZero() {
		this.moveTo(WristPosition.ZERO);
	}

	public void moveToUp() {
		this.moveTo(WristPosition.UP);
	}

	public void moveToMinus180() {
		this.moveTo(WristPosition.MINUS180);
	}

	public void moveToForwardExchange() {
		this.moveTo(WristPosition.FORWARDEXCHANGE);
	}

	public void moveToReverseExhange() {
		this.moveTo(WristPosition.REVERSEEXCHANGE);
	}

	public void moveTo(WristPosition position) {
		this.adjustWristPosition(position.getValue());
	}

	private void adjustWristPosition(int position) {
		// 3937 is the difference between the practice and comp encoder
		Wrist.wristSetpoint = RobotMap.PRACTICEBOT ? position + 3937 : position;
	}

	public static void setSetPoint(int setpoint) {
		Wrist.wristSetpoint = setpoint;
	}

	public static int getSetPoint() {
		return Wrist.wristSetpoint;
	}

}