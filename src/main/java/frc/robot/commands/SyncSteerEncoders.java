// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SyncSteerEncoders extends CommandBase {
    private Timer m_smallTimer = new Timer();
    private Timer m_permanantTimer = new Timer();

    private int state = 0;

    /** Creates a new SyncSteerEncoders. */
    public SyncSteerEncoders() {
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_smallTimer.reset();
        m_permanantTimer.reset();
    
        m_smallTimer.start();
        m_permanantTimer.start();

        state = 0;
        
        BlingSubsystem.getINSTANCE().noEncodersSynced();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("SyncSteerState", state);
        if (state == 0 && m_permanantTimer.hasElapsed(1)) {
            state = 1;
            DriverStation.reportWarning(
                    String.format("Starting to sync steer encoders (%.1fs)", m_permanantTimer.get()), false);
        }
        if (state == 1 && m_smallTimer.hasElapsed(Config.Swerve.ENCODER_SYNCING_PERIOD)) {
            m_smallTimer.restart();

            if (SwerveSubsystem.getInstance().checkSteeringEncoders() == false) {
                DriverStation.reportWarning(
                    String.format("Steering encoders are not synced, attempting to sync them... (%.1fs)", m_permanantTimer.get()),
                    false);
                SwerveSubsystem.getInstance().resetEncodersFromCanCoder();
            } else {
                state = 99;
                m_smallTimer.restart();
            }
        }
        // if (state == 2 && m_smallTimer.hasElapsed(1)) {
        //     if (SwerveSubsystem.getInstance().checkSteeringEncoders() == false) {
        //         SwerveSubsystem.getInstance().resetEncodersFromCanCoder();
        //     }
        //     state = 99;
        // }
        SmartDashboard.putNumber("SyncSteerState", state);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (SwerveSubsystem.getInstance().checkSteeringEncoders()) {
            DriverStation.reportWarning(
                        String.format("Steering encoders are synced (%.1f) \n", m_permanantTimer.get()),
                        false);
            BlingSubsystem.getINSTANCE().steerEncodersSynced();

            new WaitCommand(1).andThen(
                Commands.runOnce(() -> BlingSubsystem.getINSTANCE().steerEncodersSynced()),
                new WaitCommand(1),
                Commands.runOnce(() -> BlingSubsystem.getINSTANCE().steerEncodersSynced())
            ).schedule();
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
        return state == 99;
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
