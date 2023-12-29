// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.photonSubsystem;

//class
public class PhotonWaitForData extends CommandBase {

  int id = -1;
  public PhotonWaitForData(int desiredId) {
    addRequirements(photonSubsystem.getInstance());
    id = desiredId;
  }
  public PhotonWaitForData() {
    addRequirements(photonSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    photonSubsystem.getInstance().reset(id);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (photonSubsystem.getInstance().hasData());
  }
}
