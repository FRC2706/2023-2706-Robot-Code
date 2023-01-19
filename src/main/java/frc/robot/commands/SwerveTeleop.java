// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.config.Config.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleop extends CommandBase {
    private Joystick driverStick;
    private double teleopSpeed;
    private double teleopAngularSpeed;
    /*private SlewRateLimiter transLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);*/
    

    /** Creates a new DriveCommand. */
    public SwerveTeleop(Joystick driverStick, double teleopSpeed, double teleopAngularSpeed) {
        this.driverStick = driverStick;
        this.teleopAngularSpeed = teleopAngularSpeed;
        this.teleopSpeed = teleopSpeed;
        addRequirements(SwerveSubsystem.getInstance());

    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SwerveSubsystem.getInstance().resetLastAngles();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        double x = -1 * driverStick.getRawAxis(XboxController.Axis.kLeftY.value); 
        double y = -1 * driverStick.getRawAxis(XboxController.Axis.kLeftX.value);
        double rot = -1 * driverStick.getRawAxis(XboxController.Axis.kRightX.value);

        x = MathUtil.applyDeadband(x, Config.DRIVER_JOYSTICK_DEADBAND);
        y = MathUtil.applyDeadband(y, Config.DRIVER_JOYSTICK_DEADBAND);
        rot = MathUtil.applyDeadband(rot, Config.DRIVER_JOYSTICK_DEADBAND);

        /*x = transLimiter.calculate(x);
        y = strafeLimiter.calculate(y);
        rot = rotationLimiter.calculate(rot);*/
    
        
        SwerveSubsystem.getInstance().drive(
            x * teleopSpeed,
            y * teleopSpeed,
            rot * teleopAngularSpeed,
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