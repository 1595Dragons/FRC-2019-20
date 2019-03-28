package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Mittens extends Subsystem {

	private final DoubleSolenoid mittens = RobotMap.clamper;

	public static boolean isSecured = true;

	public Mittens() {
		super("Mittens");
	}

	public Mittens(String name) {
		super(name);
	}

	@Override
	public void initDefaultCommand() {

	}

	public void secure() {
		this.mittens.set(value);
	}

}
