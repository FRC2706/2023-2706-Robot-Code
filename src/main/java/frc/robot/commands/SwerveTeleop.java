// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleop extends CommandBase {
    private final double DEADBAND_VALUE = 0.1;
    //Originally 0.02 by Machine Mavericks
    private final double maxAccel = 0.04;

    private Joystick driverStick;
    private double prevX = 0;
    private double prevY = 0;

    /** Creates a new DriveCommand. */
    public SwerveTeleop(Joystick driverStick) {
        this.driverStick = driverStick;
        addRequirements(SwerveSubsystem.getInstance());
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        double x = -1 * driverStick.getRawAxis(XboxController.Axis.kLeftY.value); 
        double y = driverStick.getRawAxis(XboxController.Axis.kLeftX.value);
        double rot = driverStick.getRawAxis(XboxController.Axis.kRightX.value);

        x = MathUtil.applyDeadband(x, Config.DRIVER_JOYSTICK_DEADBAND);
        y = MathUtil.applyDeadband(y, Config.DRIVER_JOYSTICK_DEADBAND);
        rot = MathUtil.applyDeadband(rot, Config.DRIVER_JOYSTICK_DEADBAND);
    
        
        SwerveSubsystem.getInstance().drive(
            x * Config.Swerve.kMaxAttainableWheelSpeed,
            y * Config.Swerve.kMaxAttainableWheelSpeed,
            rot * Config.Swerve.kMaxTeleopAngularSpeed,
            true, true);
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
