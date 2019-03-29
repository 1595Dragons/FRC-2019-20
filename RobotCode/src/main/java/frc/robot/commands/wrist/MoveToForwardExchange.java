package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveToForwardExchange extends Command {

	public MoveToForwardExchange() {
		this.requires(Robot.wristSubsystem);
		this.setTimeout(0.04d);
	}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	protected void initialize() {
		Robot.wristSubsystem.moveToForwardExchange();
	}
}