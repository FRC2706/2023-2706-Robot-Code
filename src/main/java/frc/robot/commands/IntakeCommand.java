// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends InstantCommand {  
  int position;
  IntakeSubsystem intake;

  /** Creates a new IntakeCommand. */
  //PositionId 0 means open
  //position id 1 means take cone
  //position id 2 means take cube
  public IntakeCommand(int positionId) {
    position = positionId;
    intake = IntakeSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void open (){
   intake.noPressure();
  }

public void takeCone(){
  intake.highPressure();
}

public void takeCube(){
  intake.lowPressure();
  }
}