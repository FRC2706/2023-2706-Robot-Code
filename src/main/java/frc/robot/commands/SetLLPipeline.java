// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.libLimelight.LimelightHelpers;

public class SetLLPipeline extends CommandBase {
	private final int m_pipeline;
	private final Timer m_timer = new Timer();
	/** Creates a new SetLLPipeline. */
	public SetLLPipeline(int pipeline) {
		m_pipeline = pipeline;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_timer.hasElapsed(5)) {
			LimelightHelpers.setPipelineIndex("limelight", m_pipeline);
			m_timer.reset();
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_timer.stop();;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
