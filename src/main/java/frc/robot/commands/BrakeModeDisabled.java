// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.Config;
import frc.robot.subsystems.DiffNeoSubsystem;
import frc.robot.subsystems.DiffTalonSubsystem;

public class BrakeModeDisabled extends CommandBase {
    private Timer m_timer;
    /** 
     * BrakeModeDisabled will set a Differential drive chassis to brake mode 
     * when the robot is disabled and then to coast mode a few seconds later.
     * 
     * Brake mode to make sure the robot stops moving after being disabled.
     * Coast mode after a few seconds to make it easier to push around.
     */
    public BrakeModeDisabled() {
        m_timer = new Timer();
        
        // Set the motors to coast mode when the robot first boots up
        if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffNeoSubsystem)) {
            DiffNeoSubsystem.getInstance().setIdleMode(IdleMode.kCoast);
  
        } else if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffTalonSubsystem)) {
            DiffTalonSubsystem.getInstance().setNeutralMode(NeutralMode.Coast);
        
        }
    }

    /**
     * Allow this command to run when disabled.
     */
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

        if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffNeoSubsystem)) {
            DiffNeoSubsystem.getInstance().setIdleMode(IdleMode.kBrake);

        } else if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffTalonSubsystem)) {
            DiffTalonSubsystem.getInstance().setNeutralMode(NeutralMode.Brake);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();

        if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffNeoSubsystem)) {
            DiffNeoSubsystem.getInstance().setIdleMode(IdleMode.kCoast);

        } else if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffTalonSubsystem)) {
            DiffTalonSubsystem.getInstance().setNeutralMode(NeutralMode.Coast);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.get() > Config.DIFF.BRAKE_IN_DISABLE_TIME;
    }
}
