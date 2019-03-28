package frc.robot.commands.mittens;

import edu.wpi.first.wpilibj.command.Command;

public class Secure extends Command {

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

}