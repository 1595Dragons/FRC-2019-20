package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tracking {

    public MiniPID pid;   

    public Tracking(double p, double i, double d) {
        pid = new MiniPID(p, i, d);
        pid.setOutputLimits(-1, 1);
    }

    public void trackTurn(TalonSRX leftMotor, TalonSRX rightMotor, double error) {
        double power = pid.getOutput(error);
        SmartDashboard.putNumber("PID power", power);

        rightMotor.set(ControlMode.PercentOutput, -power);
        leftMotor.set(ControlMode.PercentOutput, power);
    }

    public double trackTurnPower(double error) {
        double power = pid.getOutput(error);
        SmartDashboard.putNumber("PID power", power);
        return power;
    }

}