// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  ArmSetpoint armSetpoint;
  boolean isTop;
  double defaultAngle1 = Math.PI; // angle in radians of joint 1 in default position (vertical)
  double defaultAngle2 = Math.PI/36; // angle in radians of joint 2 in default position (angle of 5 degrees)
  double[] defaultAngles;
  double[] angles;
  double angle1;
  double angle2;
  

  /** Creates a new ArmExtend. */
  
  public ArmCommand(ArmSetpoint armSetpoint, boolean isTop) {
    this.armSetpoint = armSetpoint;
    this.isTop = isTop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angles = ArmSubsystem.getInstance().calculateAngle(ArmConfig.L1, ArmConfig.L2, armSetpoint.getX(), armSetpoint.getZ());
    angle1 = angles[0];
    angle2 = angles[1];
        
    switch(armSetpoint){
      case DEFAULT:
        if (isTop) {
          ArmSubsystem.getInstance().setTopJoint(angle2);
        }
        else {
          ArmSubsystem.getInstance().setBottomJoint(angle1);
        }
      case LOW:
        if (isTop) {
          ArmSubsystem.getInstance().setTopJoint(angle2);
        }
        else {
          ArmSubsystem.getInstance().setBottomJoint(angle1);
        }
      case MEDIUM:
      if (isTop) {
        ArmSubsystem.getInstance().setTopJoint(angle2); // - only used when two motors are involved (testing only)
      }
      else {
        ArmSubsystem.getInstance().setBottomJoint(angle1);// if level 3 the angle should be 59 degrees, if level 2 angle should be 94 degrees
      }
      case HIGH:
      if (isTop) {
        ArmSubsystem.getInstance().setTopJoint(angle2);// if level 3 the angle should be 139 degrees, if level 2 the angle should be 74 degrees
      }
      else {
        ArmSubsystem.getInstance().setBottomJoint(angle1); // - only used when two motors are involved (testing only)
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().m_bottomArm.stopMotor();
    ArmSubsystem.getInstance().m_topArm.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(angle2 - ArmSubsystem.getInstance().m_topArm.getEncoder().getPosition()) < Math.toRadians(1) ) {
      return true;
    }
    else {
      return false;
    }
  }
}
