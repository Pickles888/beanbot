package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends CommandXboxController {
    

    public Controller(int port) {
        super(port);
    }

    @Override
    public double getLeftX() {
        return super.getRawAxis(0);
    }

    @Override
    public double getLeftY() {
        return super.getRawAxis(1);
    }


    @Override
    public double getRightX() {
        return super.getRawAxis(2);
    }

    @Override
    public double getRightY() {
        return super.getRawAxis(3);
    }

    @Override
    public double getLeftTriggerAxis() {
        return super.getRawAxis(5);
    }
    @Override
    public double getRightTriggerAxis() {
        return super.getRawAxis(4);
    }

}
