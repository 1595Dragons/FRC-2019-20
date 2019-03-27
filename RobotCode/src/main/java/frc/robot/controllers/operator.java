package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.toggleExtension;

public class operator {
    private Joystick operator = new Joystick(1);

    Button a = new JoystickButton(this.operator, 0),
    x = new JoystickButton(this.operator, 2);

    public operator() {
        this.x.toggleWhenPressed(new toggleExtension());
    }
}