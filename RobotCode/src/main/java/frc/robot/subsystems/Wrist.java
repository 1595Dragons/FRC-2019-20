package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
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
		this.adjustWristPosition(WristPosition.ZERO);
	}

	public void moveToUp() {
		this.adjustWristPosition(WristPosition.UP);
	}

	public void moveToMinus180() {
		this.adjustWristPosition(WristPosition.MINUS180);
	}

	public void moveToForwardExchange() {
		this.adjustWristPosition(WristPosition.FORWARDEXCHANGE);
	}

	public void moveToReverseExhange() {
		this.adjustWristPosition(WristPosition.REVERSEEXCHANGE);
	}

	private void adjustWristPosition(int position) {
		// 3937 is the difference between the practice and comp encoder
		Wrist.wristSetpoint = position;
	}

	public static void setSetPoint(int setpoint) {
		Wrist.wristSetpoint = setpoint;
	}

	public static int getSetPoint() {
		return Wrist.wristSetpoint;
	}

}