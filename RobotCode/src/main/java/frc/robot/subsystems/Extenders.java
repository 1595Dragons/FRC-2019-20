package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Extend;
import frc.robot.commands.Retract;

public class Extenders extends Subsystem {

    DoubleSolenoid extender = RobotMap.extender;

    @Override
    protected void initDefaultCommand() {
    }

    public void extend() {
        this.extender.set(Value.kForward);
        Extend.isExtended = true;
        Retract.isRetracted = false;
    }

    public void retract() {
        this.extender.set(Value.kReverse);
        Extend.isExtended = false;
        Retract.isRetracted = true;
    }

    public void toggle() {
        if (Retract.isRetracted) {
            this.extend();
        } else {
            this.retract();
        }
    }

}