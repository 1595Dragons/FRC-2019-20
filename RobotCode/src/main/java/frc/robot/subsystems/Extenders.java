package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Extenders extends Subsystem {

	private final DoubleSolenoid extender = RobotMap.extender;

	public static boolean isExtended = true;  

	public Extenders() {
		// Constructor with a fixed name
		super("Extender");
	}

	public Extenders(String name) {
		// Constructor with a dynamic name
		super(name);
	}

	@Override
	protected void initDefaultCommand() {
	}

	public void extend() {
		this.extender.set(Value.kForward);
		Extenders.isExtended = true;
	}

	public void retract() {
		this.extender.set(Value.kReverse);
		Extenders.isExtended = false;
	}

	public void toggle() {
		if (Extenders.isExtended) {
			this.retract();
		} else {
			this.extend();
		}
	}

}