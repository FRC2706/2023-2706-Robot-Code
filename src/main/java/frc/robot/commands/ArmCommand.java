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
  boolean level1;
  boolean level2;
  boolean level3;
  double integralMinimum = -0.5;
  double integralMaximum = 0.5;
  double z;
  double nodeX;
  double armSpeed = 0.2;
  /** Creates a new ArmExtend. */
  
  public ArmCommand(boolean level1, boolean level2, boolean level3) {
    this.level1 = level1;
    this.level2 = level2;
    this.level3 = level3;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystem.getInstance().m_pidControllerBottomArm.setPositionPIDWrappingMaxInput(integralMaximum);
    ArmSubsystem.getInstance().m_pidControllerBottomArm.setPositionPIDWrappingMinInput(integralMinimum);
    ArmSubsystem.getInstance().m_pidControllerTopArm.setPositionPIDWrappingMaxInput(integralMaximum);
    ArmSubsystem.getInstance().m_pidControllerTopArm.setPositionPIDWrappingMinInput(integralMinimum);
    if (level1==true) {
      z = 6;//constants --> first and second z positions - depends on the height of the node we are going for
      nodeX = 8;//the x positiion of the node we are going for
    } 
    else if(level2==true) {
      z = 30;
      nodeX = 26;
    }
    else {
      z = 36;
      nodeX = 40;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double x = ArmSubsystem.getInstance().setx(0,nodeX);
    double[] angles = ArmSubsystem.getInstance().calculateAngle(Config.Arm.L1, Config.Arm.L2, x, z);
    double angle1 = angles[0];
    double angle2 = angles[1];

    double targetAngularDistanceBottomArm = ArmSubsystem.getInstance().getAngularDistance(angle1, Config.Arm.NEO_GEAR_RATIO);
    double targetAngularDistanceTopArm = ArmSubsystem.getInstance().getAngularDistance(angle2, Config.Arm.NEO_GEAR_RATIO);
    ArmSubsystem.getInstance().setJoint1(targetAngularDistanceBottomArm);
    // if (ArmSubsystem.getInstance().m_absoluteBottomArmEncoder.getPosition() < targetAngularDistanceBottomArm) {
    //   ArmSubsystem.getInstance().setJoint1(targetAngularDistanceBottomArm);
    // }
    // else {
    //   ArmSubsystem.getInstance().m_bottomArm.set(0);
    // }

    // if (ArmSubsystem.getInstance().m_absoluteTopArmEncoder.getPosition() < targetAngularDistanceTopArm) {
    //   ArmSubsystem.getInstance().setJoint2(targetAngularDistanceTopArm);
    // }
    // else {
    //   ArmSubsystem.getInstance().m_topArm.set(0);
    // }

    System.out.println(Arrays.toString(angles));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
