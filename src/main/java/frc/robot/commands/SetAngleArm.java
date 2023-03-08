// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetAngleArm extends CommandBase {

  double angle;
  boolean m_slowerAcceleration;
  boolean isTop;
  /** Creates a new SetAngleArm. */
  public SetAngleArm(double degAngle, boolean m_slowerAcceleration, boolean isTop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = Math.toRadians(degAngle);
    this.m_slowerAcceleration = m_slowerAcceleration;
    this.isTop = isTop;


    // addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isTop) {
      ArmSubsystem.getInstance().setConstraintsTop(m_slowerAcceleration);
    }
    else {
      ArmSubsystem.getInstance().setConstraintsBottom(m_slowerAcceleration);
    }
    ArmSubsystem.getInstance().resetMotionProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isTop) {
      ArmSubsystem.getInstance().setTopJoint(angle);
    }
    else {
      ArmSubsystem.getInstance().setBottomJoint(angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
