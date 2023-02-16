// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Arrays;

public class ArmCommand extends CommandBase {
  Level level = Level.LOW;
  boolean isTop;
  double integralMinimum = -0.5;
  double integralMaximum = 0.5;
  double z;
  double nodeX;
  double armSpeed = 0.2;
  double defaultAngle1 = Math.PI; // angle in radians of joint 1 in default position (vertical)
  double defaultAngle2 = Math.PI/36; // angle in radians of joint 2 in default position (angle of 5 degrees)
  double[] defaultAngles;
  double targetAngularDistanceBottomArm;
  double targetAngularDistanceTopArm;
  double[] angles;
  double angle1;
  double angle2;
  public enum Level {
    DEFAULT,
    LOW,
    MEDIUM,
    HIGH
  }

  /** Creates a new ArmExtend. */
  
  public ArmCommand(Level level, boolean isTop) {
    this.level = level;
    this.isTop = isTop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nodeX = 0;
    z = 0;
    switch(level) {
      case DEFAULT:
        defaultAngles = new double[]{defaultAngle1, defaultAngle2};
        z = 0;
        nodeX = 0;
        break;
      case LOW:
        z = 4;//constants --> first and second z positions - depends on the height of the node we are going for
        nodeX = 19.382;//the x positiion of the node we are going for
        break;
      case MEDIUM:
        z = 24;
        nodeX = 37.132;
        break;
      case HIGH:
        z = 35;
        nodeX = 54.132;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double x = ArmSubsystem.getInstance().setx(0,nodeX);
    angles = ArmSubsystem.getInstance().calculateAngle(Config.Arm.L1, Config.Arm.L2, x, z);
    angle1 = angles[0];
    angle2 = angles[1];

    targetAngularDistanceBottomArm = ArmSubsystem.getInstance().getAngularDistance(angle1, Config.Arm.NEO_GEAR_RATIO);
    targetAngularDistanceTopArm = ArmSubsystem.getInstance().getAngularDistance(angle2, Config.Arm.NEO_GEAR_RATIO);
    
    switch(level){
      case DEFAULT:
        ArmSubsystem.getInstance().setDefault(defaultAngles);
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
