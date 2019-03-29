package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.wrist.WristPosition;

public class Wrist extends Subsystem {

	public static int wristSetpoint; // TODO Adjust wrist here becasue of practice offset (+3937)

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
		Wrist.wristSetpoint = WristPosition.ZERO.getValue();
	}

	public void moveToUp() {
		Wrist.wristSetpoint = WristPosition.UP.getValue();
	}

	public void moveToMinus180() {
		Wrist.wristSetpoint = WristPosition.MINUS180.getValue();
	}

	public void moveToForwardExchange() {
		Wrist.wristSetpoint = WristPosition.FORWARDEXCHANGE.getValue();
	}

	public void moveToReverseExhange() {
		Wrist.wristSetpoint = WristPosition.REVERSEEXCHANGE.getValue();
	}

	public void moveTo(WristPosition position) {
		Wrist.wristSetpoint = position.getValue();
	}

}