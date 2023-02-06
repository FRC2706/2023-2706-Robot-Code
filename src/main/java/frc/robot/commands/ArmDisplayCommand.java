// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmDisplay;

public class ArmDisplayCommand extends CommandBase {
    CommandXboxController controller;
    ArmDisplay armDisplay;

    private final double length1 = 26.2;
    private final double length2 = 37;

    private final double joystickSpeed = 0.4;

    double x = 0.2;
    double z = 0.2;

    /** Creates a new ArmDisplayCommand. */
    public ArmDisplayCommand(CommandXboxController controller) {
        this.controller = controller;

        armDisplay = new ArmDisplay(length1, length2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        x = 20;
        z = 20;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        x += controller.getRawAxis(0) * joystickSpeed;
        z += -controller.getRawAxis(1) * joystickSpeed;

        double[] xz = armDisplay.calculateAngle(length1, length2, x, z);

        armDisplay.updateDisplay(xz[0], xz[1]);

        System.out.printf("X:%.1f, Z:%.1f\n", x, z);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
