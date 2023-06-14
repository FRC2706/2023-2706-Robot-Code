// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffTalonSubsystem;

public class limelightrotate extends CommandBase {
  NetworkTableEntry txpub; 
  NetworkTableEntry typub;
  /** Creates a new limelightrotate. */
  public limelightrotate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DiffTalonSubsystem.getInstance());
    txpub=NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    typub=NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value=txpub.getDouble(0);
    value*=-0.05;
    value+=Math.copySign(0.01, value);
    double limelightMountingDegrees = 11.0;
    double limelightLenHeightMeters = 0.34;
    double angleToGoalDegrees = typub.getDouble(0)+limelightMountingDegrees;
    double angleToGoalRadians = angleToGoalDegrees*(3.14159/180);
    double Distance = (1.26-limelightLenHeightMeters)/Math.tan(angleToGoalRadians);
    double ForwardVal = (Distance-1.5)*0.87;
    DiffTalonSubsystem.getInstance().arcadeDrive(ForwardVal, txpub.getDouble(0)*-0.05);
    System.out.println(Distance);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DiffTalonSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
