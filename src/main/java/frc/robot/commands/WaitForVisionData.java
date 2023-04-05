// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionNTSubsystem;

public class WaitForVisionData extends CommandBase {
    private final int NUM_DATA_THRESHOLD = 15;
    private final double TIME_WITH_NO_DATA = 1.0;

    private boolean m_isTapeNotApril;
    private Timer m_timer = new Timer();
    private int numGoodData;

    /** Creates a new WaitForVisionData. */
    public WaitForVisionData(boolean isTapeNotApril) {
        m_isTapeNotApril = isTapeNotApril;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        numGoodData = 0;
        m_timer.restart();

        if (m_isTapeNotApril) {
            VisionNTSubsystem.getInstance().resetTapeFilters();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d visionData;
        if (m_isTapeNotApril) {
            visionData = VisionNTSubsystem.getInstance().calculateTapeTarget();
        } else {
            visionData = VisionNTSubsystem.getInstance().calculateAprilTarget();
        }

        if (visionData != null) {
            numGoodData++;
            m_timer.restart();
        } else if (m_timer.hasElapsed(TIME_WITH_NO_DATA)) {
            numGoodData = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return numGoodData > NUM_DATA_THRESHOLD;
    }
}
