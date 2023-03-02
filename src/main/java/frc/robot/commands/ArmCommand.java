// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmDisplay;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  ArmSetpoint armSetpoint;
  double defaultAngle1 = Math.PI; // angle in radians of joint 1 in default position (vertical)
  double defaultAngle2 = Math.PI/36; // angle in radians of joint 2 in default position (angle of 5 degrees)
  double[] defaultAngles;
  double[] angles;
  double angle1;
  double angle2;
  private final boolean m_slowerAcceleration;

  // arm simulation variables
  CommandXboxController controller;
  ArmDisplay armDisplay;

  double x = 0.2;
  double z = 0.2;
  

  /** Creates a new ArmExtend. */
  
  public ArmCommand(ArmSetpoint armSetpoint, boolean slowerAcceleration) {
    this.armSetpoint = armSetpoint;
    this.m_slowerAcceleration = slowerAcceleration;
    // Use addRequirements() here to declare subsystem dependencies.
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
    angles = ArmSubsystem.getInstance().inverseKinematics(ArmConfig.L1, ArmConfig.L2, armSetpoint.getX(), armSetpoint.getZ());
    angle1 = angles[0];
    angle2 = angles[1];

    ArmSubsystem.getInstance().setTopJoint(angle2);
    // ArmSubsystem.getInstance().setBottomJoint(angle1);

    armDisplay.updateMeasurementDisplay(angle1, angle2);
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
