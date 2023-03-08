// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfig;
import frc.robot.subsystems.ArmSubsystem;

public class SyncArmEncoders extends CommandBase {
    private Timer m_smallTimer = new Timer();
    private Timer m_permanantTimer = new Timer();
    private boolean m_needsSyncing = true;
    private int m_numSamples = 0;
    private double m_sumBotSamples = 0;
    private double m_sumTopSamples = 0;

    /** Creates a new SyncSteerEncoders. */
    public SyncArmEncoders() {
        addRequirements(ArmSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_smallTimer.restart();
        m_permanantTimer.restart();
        m_numSamples = 0;
        m_sumBotSamples = 0;
        m_sumTopSamples = 0;
        m_needsSyncing = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_smallTimer.get() > ArmConfig.ENCODER_SYNCING_PERIOD) {
            m_smallTimer.reset();

            if (m_needsSyncing) {
                if (ArmSubsystem.getInstance().areEncodersSynced() == false) {
                    m_needsSyncing = true;
                    DriverStation.reportWarning(
                        String.format("Arm encoders are not synced, attempting to sync them... (%.1fs)", m_permanantTimer.get()),
                        false);
                    ArmSubsystem.getInstance().updateFromAbsoluteBottom();
                    ArmSubsystem.getInstance().updateFromAbsoluteTop();
                } else {
                    m_needsSyncing = false;
                }
            }
        }
        if (m_needsSyncing == false) {
            m_sumBotSamples += ArmSubsystem.getInstance().getAbsoluteBottom();
            m_sumTopSamples += ArmSubsystem.getInstance().getAbsoluteTop();
            m_numSamples++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (m_needsSyncing && m_numSamples > 2) {
            DriverStation.reportWarning(
                        String.format("Arm encoders are synced (%.1f) \n", m_permanantTimer.get()),
                        false);

            ArmSubsystem.getInstance().resetEncoder(
                m_sumBotSamples / m_numSamples,
                m_sumTopSamples / m_numSamples
            );
        }

        m_permanantTimer.stop();
        m_smallTimer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_permanantTimer.get() > ArmConfig.ENCODER_SYNCING_TIMEOUT) {
            DriverStation.reportError(
                String.format("Arm encoders are not synced. SyncArmEncoders spent %.1fs trying to sync them and has timed out",
                m_permanantTimer.get()),
                false);

            return true;
        }

        return ArmSubsystem.getInstance().areEncodersSynced() && m_numSamples > ArmConfig.NUM_SYNCING_SAMPLES;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        DriverStation.reportWarning("Another ArmSubsystem command was scheduled while " +
                "SyncArmEncoders is still running. Cancelling the incoming command.", false);
        return InterruptionBehavior.kCancelIncoming;
    }
}
