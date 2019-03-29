package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;

public class MoveToForwardExchange extends Command { // TODO

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
    
}