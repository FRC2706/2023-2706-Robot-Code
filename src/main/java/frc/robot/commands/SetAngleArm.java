// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetAngleArm extends CommandBase {

  double angle;
  boolean m_slowerAcceleration;
  /** Creates a new SetAngleArm. */
  public SetAngleArm(double angle, boolean m_slowerAcceleration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.m_slowerAcceleration = m_slowerAcceleration;


    addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystem.getInstance().setConstraints(m_slowerAcceleration);
    ArmSubsystem.getInstance().resetMotionProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.getInstance().setTopJoint(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ArmSubsystem.getInstance().m_bottomArm.stopMotor();
    ArmSubsystem.getInstance().m_topArm.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
