// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.libLimelight.LimelightHelpers;
import frc.robot.subsystems.DiffTalonSubsystem;

public class limelightrotate extends CommandBase {
  NetworkTableEntry txpub; 
  NetworkTableEntry typub;
  /** Creates a new limelightrotate. */
  public limelightrotate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DiffTalonSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!LimelightHelpers.getTV("")){
      DiffTalonSubsystem.getInstance().stopMotors();
    } else{
      double value=LimelightHelpers.getTX("");
      value*=-0.05;
      value+=Math.copySign(0.01, value);
      double Distance = (1.43 / (LimelightHelpers.getTLONG("") / 52));
      double ForwardVal = (Distance-1.5)*0.87;
      ForwardVal+=Math.copySign(0.06, ForwardVal);
      DiffTalonSubsystem.getInstance().arcadeDrive(ForwardVal, value);
      System.out.println(Distance);
    }
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
