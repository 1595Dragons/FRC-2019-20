package frc.robot.commands.mittens;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Secure extends Command {

    public Secure() {
        this.requires(Robot.mitten);
        this.setTimeout(0.25d);
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

    protected void initialize() {
        Robot.mitten.secure();
    }

    protected void execute() {
        // Do nothing for now
    }

}