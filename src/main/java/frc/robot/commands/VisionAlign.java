// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionAlign extends CommandBase {
  /** Creates a new VisionAlign. */

  private SwerveSubsystem swerve;

  private double error, lastError, lastTime, currentTime, dt, rotationRate, rotationSum, currentPosition;

  private double target = 144; // fweioajfweio;aj
  private double pixel_offset = 4;
  private double kP = 0;
  private double kI = 0;
  private double iLimit = 0;
  private double kD = 0;

  private NetworkTableEntry DistanceToTarget = table.getEntry("DistanceToTarget");
  private NetworkTableEntry YawToTarget = table.getEntry("YawToTarget");
  private NetworkTableEntry TargetPixelFromCenter = table.getEntry("TargetPixelFromCenter");

  public VisionAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = 0;
    lastError = 0;
    lastTime = 0;
    currentTime = 0;
    rotationRate = 0;
    rotationSum = 0;
    dt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = TargetPixelFromCenter;
    currentTime = Timer.getFPGATimestamp();
    dt = currentTime - lastTime;

    //P Calculation
    error = target - currentPosition;

    //I Calculation
    if(Math.abs(error)<iLimit){
      rotationSum += error*dt;
    
    rotationRate = (error - lastError) / dt;

    double output = kP*error + kI*rotationSum + kD*rotationRate;
    swerve.drive(0, 0, output, false, true);

    lastError = error;
    lastTime = currentTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((currentPosition < (target + pixel_offset)) && (currentPosition > (target - pixel_offset))) || currentPosition == -1;
  }
}
