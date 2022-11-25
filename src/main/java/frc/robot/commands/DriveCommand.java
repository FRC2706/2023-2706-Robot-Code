// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final double DEADBAND_VALUE = 0.1;
    //Originally 0.02 by Machine Mavericks
    private final double maxAccel = 0.04;

    private Joystick driverStick;
    private double prevX = 0;
    private double prevY = 0;

    /** Creates a new DriveCommand. */
    public DriveCommand(Joystick driverStick) {
        this.driverStick = driverStick;
        addRequirements(DriveSubsystem.getInstance());
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        double newX = driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_Y);
        double newY = driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_X);
        double newRot = driverStick.getRawAxis(Config.RIGHT_CONTROL_STICK_X);

        if (Math.abs(newX) < DEADBAND_VALUE) {
            newX = 0;
        }

        if (Math.abs(newY) < DEADBAND_VALUE) {
            newY = 0;
        }

        if (Math.abs(newRot) < DEADBAND_VALUE) {
            newRot = 0;
        }

        newX = (newX - prevX) > maxAccel ? prevX + maxAccel : newX;
        newX = (newX - prevX) < -1 * maxAccel ? prevX - maxAccel : newX;

        newY = (newY - prevY) > maxAccel ? prevY + maxAccel : newY;
        newY = (newY - prevY) < -1 * maxAccel ? prevY - maxAccel : newY;

        if (Math.abs(newX) < DEADBAND_VALUE && Math.abs(newY) < DEADBAND_VALUE && Math.abs(newRot) < DEADBAND_VALUE) {
            DriveSubsystem.getInstance().stopMotors();
        } else {
            DriveSubsystem.getInstance().drive(
                newX * Config.kMaxAttainableWheelSpeed,
                newY * Config.kMaxAttainableWheelSpeed,
                newRot * Config.kMaxTeleopAngularSpeed,
                true);
        }

        prevX = newX;
        prevY = newY;
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
