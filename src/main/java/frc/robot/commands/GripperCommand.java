// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;
import frc.robot.subsystems.GripperSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GripperCommand extends InstantCommand {  
  int position;

  Consumer <RobotGamePieceState> newRobotState;
  GripperSubsystem gripper;

  /** Creates a new IntakeCommand. */
  //PositionId -1 means using intake vision
  //PositionId 1 means open
  //position id 2 means take cone
  //position id 3 means take cube
  public GripperCommand(int positionId, Consumer<RobotGamePieceState> robotState) {
    position = positionId;
    newRobotState = robotState;
    gripper = GripperSubsystem.getInstance();
    if (gripper!=null){
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripper[]);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  switch(position){
     case 1:
      open();
      newRobotState.accept(RobotGamePieceState.NoGamePiece);
      break;
     case 2:
      takeCone();
      newRobotState.accept(RobotGamePieceState.HasCone);
      break;
     case 3:
      takeCube();
      newRobotState.accept(RobotGamePieceState.HasCube);
      break;
     default:
      break;
     }
  }

  public void open (){
   if (gripper!=null)
    gripper.noPressure();
  }

  public void takeCone(){
   if (gripper!=null)
    gripper.highPressure();
}

  public void takeCube(){
   if (gripper!=null)
    gripper.lowPressure();
  }
}