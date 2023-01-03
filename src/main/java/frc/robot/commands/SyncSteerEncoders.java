// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class SyncSteerEncoders extends CommandBase {
    private Timer m_smallTimer = new Timer();
    private Timer m_permanantTimer = new Timer();
    private boolean m_needsSyncing = false;

    private double m_timeToSync[] = new double[]{-1, -1, -1, -1}; 

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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updateArr();
        if (m_smallTimer.get() > Config.Swerve.ENCODER_SYNCING_PERIOD) {
            m_smallTimer.reset();
            SwerveSubsystem sub = SwerveSubsystem.getInstance();
            if (SwerveSubsystem.getInstance().checkSteeringEncoders() == false) {
                m_needsSyncing = true;
                DriverStation.reportWarning(
                    String.format("\nSteering encoders are not synced, attempting to sync them... (%.1fs) \nFrontLeft: %s\nFrontRight: %s\nRearLeft: %s\nRearRight: %s\n",
                    m_permanantTimer.get(), sub.check(0), sub.check(1), sub.check(2), sub.check(3)),
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

        updateArr();
        SwerveSubsystem sub = SwerveSubsystem.getInstance();
        NetworkTableEntry testNum = NetworkTableInstance.getDefault().getTable("testing").getEntry("TestCount");
        testNum.setPersistent();
        testNum.setNumber(-1967);
        try {
            FileWriter fstream = new FileWriter("/U/data_captures/TestingSyncSteerEncoders.csv", true);
            BufferedWriter log_file = new BufferedWriter(fstream);

            log_file.write(
                String.format(
                    "%d, %.4f, %.4f, %.4f, %.4f, \n",
                    testNum.getNumber(-1967).intValue(),
                    m_timeToSync[0],
                    m_timeToSync[1],
                    m_timeToSync[2],
                    m_timeToSync[3]));

            log_file.close();
            testNum.setNumber(testNum.getNumber(-1).intValue()+1);
            System.out.println("CSV successful written to");

        } catch (IOException e) {
            System.out.println("AAAAAAAAAAAAAAAAAAAAA");
        }
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

    private void updateArr() {
        for (int i = 0; i < 4; i++) {
            if (m_timeToSync[i] == -1) {
                if (SwerveSubsystem.getInstance().check(i)) {
                    m_timeToSync[i] = 0;
                } else {
                    m_timeToSync[i] = -2;
                }
            } else {
                if (m_timeToSync[i] == -2 && SwerveSubsystem.getInstance().check(i)) {
                    m_timeToSync[i] = m_permanantTimer.get();
                }
            }
        }

    }
}
