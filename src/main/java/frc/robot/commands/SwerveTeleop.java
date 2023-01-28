// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleop extends CommandBase {
    private CommandXboxController driverStick;
    private double teleopSpeed;
    private double teleopAngularSpeed;
    private SlewRateLimiter transLimiter; 
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter rotationLimiter;
    

    /** Creates a new DriveCommand. */
    public SwerveTeleop(CommandXboxController driverStick, double teleopSpeed, double teleopAngularSpeed, double rateLimit) {
        this.driverStick = driverStick;
        this.teleopAngularSpeed = teleopAngularSpeed;
        this.teleopSpeed = teleopSpeed;
        transLimiter = new SlewRateLimiter(rateLimit);
        strafeLimiter = new SlewRateLimiter(rateLimit);
        rotationLimiter = new SlewRateLimiter(rateLimit);
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
        SwerveSubsystem.getInstance().drive(
            calculateX(),
            calculateY(),
            calculateRot(),
            true, true);
    }

    protected double calculateX(){
        double x = -1 * driverStick.getRawAxis(XboxController.Axis.kLeftY.value);
        x = MathUtil.applyDeadband(x, Config.DRIVER_JOYSTICK_DEADBAND);
        if (x==0){
            return 0;
        }
        x = transLimiter.calculate(x);
        x *= teleopSpeed;
        return x;
    }

    protected double calculateY(){
        double y = -1 * driverStick.getRawAxis(XboxController.Axis.kLeftX.value);
        y = MathUtil.applyDeadband(y, Config.DRIVER_JOYSTICK_DEADBAND);
        if (y==0){
            return 0;
        }
        y = strafeLimiter.calculate(y);
        y *= teleopSpeed;
        return y;
    }

    protected double calculateRot(){
        double rot = -1 * driverStick.getRawAxis(XboxController.Axis.kRightX.value);
        rot = MathUtil.applyDeadband(rot, Config.DRIVER_JOYSTICK_DEADBAND);
        if (rot == 0){
            return 0;
        }
        rot = rotationLimiter.calculate(rot);
        rot *= teleopAngularSpeed;
        return rot;

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