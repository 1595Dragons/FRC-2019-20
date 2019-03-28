package frc.robot.commands.mittens;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Mittens;

public class toggleMitten extends Command {

    public toggleMitten() {
        this.requires(Robot.mitten);
        this.setTimeout(0.25d);
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

    protected void initialize() {
		if (Mittens.isSecured) {
			Robot.mitten.release();
		} else {
			Robot.mitten.secure();
		}
	}

	protected void execute() {
		// Do nothing for now
	}

}