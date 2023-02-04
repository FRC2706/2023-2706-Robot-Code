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
  //PositionId 1 means open
  //position id 2 means take cone
  //position id 3 means take cube
  public IntakeCommand(int positionId) {
    position = positionId;
    intake = IntakeSubsystem.getInstance();
    if (intake!=null){
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  switch(position){
     case 1:
      open();
      break;
     case 2:
      takeCone();
      break;
     case 3:
      takeCube();
      break;
     default:
      break;
     }
  }

  public void open (){
   if (intake!=null)
    intake.noPressure();
  }

  public void takeCone(){
   if (intake!=null)
    intake.highPressure();
}

  public void takeCube(){
   if (intake!=null)
    intake.lowPressure();
  }
}