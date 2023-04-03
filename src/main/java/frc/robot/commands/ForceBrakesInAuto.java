// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ForceBrakesInAuto extends CommandBase {
    Timer timer = new Timer();
    
    /** Creates a new ForceBrakesInAuto. */
    public ForceBrakesInAuto() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted == false) {
            
            // Hog requirements for one cycle to cancel auto commands
            Commands.runOnce(() -> ArmSubsystem.getInstance(), ArmSubsystem.getInstance());
            Commands.runOnce(() -> SwerveSubsystem.getInstance(), SwerveSubsystem.getInstance());
            
            // Turn the brakes on
            ArmSubsystem.getInstance().controlBottomArmBrake(true);
            ArmSubsystem.getInstance().controlTopArmBrake(true);
        }
        
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Swerve.TIME_FOR_BRAKES_IN_AUTO);
    }
}
