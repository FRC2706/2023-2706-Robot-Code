// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.GripperSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GripperCommand extends InstantCommand {  
  public enum GRIPPER_INSTRUCTION {
    USE_VISION,
    OPEN,
    PICK_UP_CONE,
    PICK_UP_CUBE
  }
  GRIPPER_INSTRUCTION m_gripperInstruction;

  Consumer <RobotGamePieceState> newRobotState;
  GripperSubsystem gripper;
  BlingSubsystem bling;
  
  /** Creates a new IntakeCommand. */
  public GripperCommand(GRIPPER_INSTRUCTION gripperInstruction, Consumer<RobotGamePieceState> robotState) {
    m_gripperInstruction = gripperInstruction;
    newRobotState = robotState;
    gripper = GripperSubsystem.getInstance();
    if (gripper!=null){
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripper);
    }

    bling = BlingSubsystem.getINSTANCE();
    if (bling != null) {
      addRequirements(bling);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  switch(m_gripperInstruction){
     case USE_VISION:
     //@todo: read the network table, decide to take cone/cube/open/do nothing
     break;
     case OPEN:
      open();
      newRobotState.accept(RobotGamePieceState.NoGamePiece);
      break;
     case PICK_UP_CONE:
      takeCone();
      newRobotState.accept(RobotGamePieceState.HasCone);
      break;
     case PICK_UP_CUBE:
      takeCube();
      newRobotState.accept(RobotGamePieceState.HasCube);
      break;
     default:
     //?? for default??
      break;
     }
  }

  public void open (){
   if (gripper!=null) {
    gripper.noPressure();
    //set bling disabled
    if (bling != null) {
      bling.setDisabled();
    }
   }
  }

  public void takeCone(){
   if (gripper!=null)
    gripper.highPressure();
    //set bling flashing yellow
    if(bling != null) {
      bling.setYellow();;
    }
}

  public void takeCube(){
   if (gripper!=null)
    gripper.lowPressure();
    //set bling flashing purple
    if(bling != null) {
      bling.setPurple();
    }
  }
}