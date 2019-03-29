package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;

public class MoveToReverseExchange extends Command { // TODO

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

}