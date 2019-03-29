package frc.robot.commands.mittens;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Release extends Command {

	public Release() {
		this.requires(Robot.mitten);
		this.setTimeout(0.02d);
	}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	protected void initialize() {
		Robot.mitten.release();
	}

	protected void execute() {
		// Do nothing for now
	}

}