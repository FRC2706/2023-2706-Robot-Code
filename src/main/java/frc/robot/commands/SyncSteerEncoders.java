// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class SyncSteerEncoders extends CommandBase {
    private Timer m_smallTimer = new Timer();
    private Timer m_permanantTimer = new Timer();
    private boolean m_needsSyncing = false;

    /** Creates a new SyncSteerEncoders. */
    public SyncSteerEncoders() {
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_smallTimer.restart();
        m_permanantTimer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_smallTimer.get() > Config.Swerve.ENCODER_SYNCING_PERIOD) {
            m_smallTimer.reset();

            if (SwerveSubsystem.getInstance().checkSteeringEncoders() == false) {
                m_needsSyncing = true;
                DriverStation.reportWarning(
                    String.format("Steering encoders are not synced, attempting to sync them... (%.1fs)", m_permanantTimer.get()),
                    false);
                SwerveSubsystem.getInstance().resetEncodersFromCanCoder();
            } 
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (m_needsSyncing) {
            DriverStation.reportWarning(
                        String.format("Steering encoders are synced (%.1f) \n", m_permanantTimer.get()),
                        false);
        }

        m_permanantTimer.stop();
        m_smallTimer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_permanantTimer.get() > Config.Swerve.ENCODER_SYNCING_TIMEOUT) {
            DriverStation.reportError(
                String.format("Steering encoders are not synced. SyncSteerEncoders spent %.1fs trying to sync them and has timed out",
                m_permanantTimer.get()),
                false);

            return true;
        }

        return SwerveSubsystem.getInstance().checkSteeringEncoders();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        DriverStation.reportWarning("Another SwerveSubsystem command was scheduled while " +
                "SyncSteerEncoders is still running. Cancelling the incoming command.", false);
        return InterruptionBehavior.kCancelIncoming;
    }
}
