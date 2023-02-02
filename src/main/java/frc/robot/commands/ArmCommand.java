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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmCommand extends CommandBase {
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double integralMinimum = -0.5;
  double integralMaximum = 0.5;

  PIDController pid = new PIDController(kP, kI, kD);

  double NEO_GEAR_RATIO = 7.67;
  double L1 = 0; //length of arm 1
  double L2 =0; //length of arm 2
  double z = 0;//constants --> first and second z positions - depends on the height of the node we are going for
  double nodeX = 0; //the x positiion of the node we are going for
  double armSpeed = 0.2;


  /** Creates a new ArmExtend. */
  
  public ArmCommand() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setIntegratorRange(integralMinimum, integralMaximum);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double drivetrainx = SwerveSubsystem.getInstance().getPose().getX();
    double x = ArmSubsystem.getInstance().setx(drivetrainx,nodeX);
    double[] angles = ArmSubsystem.getInstance().calculateAngle(L1, L2, x, z);
    double angle1 = angles[0];
    double angle2 = angles[1];

    double targetAngularDistanceBottomArm = ArmSubsystem.getInstance().getAngularDistance(angle1, NEO_GEAR_RATIO);
    double targetAngularDistanceTopArm = ArmSubsystem.getInstance().getAngularDistance(angle2, NEO_GEAR_RATIO);

    if (ArmSubsystem.getInstance().m_absoluteBottomArmEncoder.getPosition() < targetAngularDistanceBottomArm) {
      ArmSubsystem.getInstance().setJoint1(targetAngularDistanceBottomArm);
    }
    else {
      ArmSubsystem.getInstance().m_bottomArm.set(0);
    }

    if (ArmSubsystem.getInstance().m_absoluteTopArmEncoder.getPosition() < targetAngularDistanceTopArm) {
      ArmSubsystem.getInstance().setJoint2(targetAngularDistanceTopArm);
    }
    else {
      ArmSubsystem.getInstance().m_topArm.set(0);
    }


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
