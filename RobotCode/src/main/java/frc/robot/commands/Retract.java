package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Retract extends Command {

	public Retract() {
		this.requires(Robot.extender);
		this.setTimeout(0.75d);
	}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	protected void initialize() {
		Robot.extender.retract();
	}

	protected void execute() {
		// Do nothing for now
	}
}