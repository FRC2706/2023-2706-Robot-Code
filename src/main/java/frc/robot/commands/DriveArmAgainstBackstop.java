// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlingSubsystem;

public class DriveArmAgainstBackstop extends CommandBase {
    Timer m_timer = new Timer();

    /** Creates a new DriveArmAgainstBackstop. */
    public DriveArmAgainstBackstop() {
        addRequirements(ArmSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();
        ArmSubsystem.getInstance().controlTopArmBrake(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double voltage = -0.23;

        if (m_timer.hasElapsed(1)) {
            voltage = 0;
        } 

        if (m_timer.hasElapsed(2)) {
            voltage = -0.03;
        }

        if (m_timer.hasElapsed(3)) {
            ArmSubsystem.getInstance().controlTopArmBrake(true);
        }

        ArmSubsystem.getInstance().setTopVoltage(voltage);      
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().controlTopArmBrake(true);
        m_timer.stop();
        ArmSubsystem.getInstance().stopMotors();

        new SequentialCommandGroup(
            new SetBlingCommand(13),
            (new WaitCommand(2)),
            (new SetBlingCommand(0))
        ).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.get() > 3.5;
    }
}
