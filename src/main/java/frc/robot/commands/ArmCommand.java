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
  int level;
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

  /** Creates a new ArmExtend. */
  
  public ArmCommand(int level, boolean isTop) {
    this.level = level;
    this.isTop = isTop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(level) {
      case 0:
        defaultAngles = new double[]{defaultAngle1, defaultAngle2};
        z = 0;
        nodeX = 0;
      case 1:
        z = 4;//constants --> first and second z positions - depends on the height of the node we are going for
        nodeX = 19.382;//the x positiion of the node we are going for
      case 2:
        z = 24;
        nodeX = 37.132;
      case 3:
        z = 35;
        nodeX = 54.132;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double x = ArmSubsystem.getInstance().setx(0,nodeX);
    double[] angles = ArmSubsystem.getInstance().calculateAngle(Config.Arm.L1, Config.Arm.L2, x, z);
    double angle1 = angles[0];
    double angle2 = angles[1];

    targetAngularDistanceBottomArm = ArmSubsystem.getInstance().getAngularDistance(angle1, Config.Arm.NEO_GEAR_RATIO);
    targetAngularDistanceTopArm = ArmSubsystem.getInstance().getAngularDistance(angle2, Config.Arm.NEO_GEAR_RATIO);
    
    switch(level){
      case 0:
        ArmSubsystem.getInstance().setDefault(defaultAngles);
      case 1:
        // ArmSubsystem.getInstance().setJoint1(angle1);
        ArmSubsystem.getInstance().setJoint2(angle2);
      case 2:
        // ArmSubsystem.getInstance().setJoint1(angle1);// if level 3 the angle should be 59 degrees, if level 2 angle should be 94 degrees
        ArmSubsystem.getInstance().setJoint2(angle2); // - only used when two motors are involved (testing only)
      case 3:
      // ArmSubsystem.getInstance().setJoint1(angle1); - only used when two motors are involved (testing only)
        ArmSubsystem.getInstance().setJoint2(angle2);// if level 3 the angle should be 139 degrees, if level 2 the angle should be 74 degrees
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
    if (isTop == true) {
      if (Math.abs(targetAngularDistanceTopArm - ArmSubsystem.getInstance().m_topArm.getEncoder().getPosition()) < Math.toRadians(1) ) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      if (Math.abs(targetAngularDistanceBottomArm - ArmSubsystem.getInstance().m_topArm.getEncoder().getPosition()) < Math.toRadians(1) ) {
        return true;
      }
      else {
        return false;
      }
    }
    
  }
}
