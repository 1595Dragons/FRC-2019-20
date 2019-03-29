package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.extender.toggleExtension;
import frc.robot.commands.mittens.toggleMitten;

public class operator {
    private Joystick operator = new Joystick(1);

    Button a = new JoystickButton(this.operator, 1), x = new JoystickButton(this.operator, 3),
            dpad_down = new POVButton(this.operator, 180), dpad_up = new POVButton(this.operator, 0),
            dpad_left = new POVButton(this.operator, 270), dpad_right = new POVButton(this.operator, 90),
            left_bumper = new JoystickButton(this.operator, 5), right_bumper = new JoystickButton(this.operator, 6);

    public operator() {

        // Hatch mechanism
        this.x.toggleWhenPressed(new toggleExtension());
        this.a.toggleWhenPressed(new toggleMitten());

        // Add commands for autowrist
        //this.dpad_left.toggleWhenPressed(new MoveToZero());
        //this.dpad_up.toggleWhenPressed(new MoveToUp());
    }
}