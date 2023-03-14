// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import javax.lang.model.util.ElementScanner14;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;
import frc.robot.subsystems.ArmSubsystem;
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
  
  BooleanSubscriber bDetectCone;
  BooleanSubscriber bDetectCube;

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

    bDetectCone = NetworkTableInstance.getDefault().getTable("MergePipelineIntake22").getBooleanTopic("DetectCone").subscribe(false);
    bDetectCube = NetworkTableInstance.getDefault().getTable("MergePipelineIntake22").getBooleanTopic("DetectCube").subscribe(false);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  switch(m_gripperInstruction){
     case USE_VISION:
     {
      // read the network table, decide to take cone/cube/open/do nothing
      //give cone a higher priority than cube
      if( bDetectCone.get() == true)
      {
        takeCone();
        if (newRobotState != null)
          newRobotState.accept(RobotGamePieceState.HasCone);
        ArmSubsystem.getInstance().setHasCone(true);
      }
      else if( bDetectCube.get() == true)
      {
        takeCube();
        if ( newRobotState != null)
          newRobotState.accept(RobotGamePieceState.HasCube);
        ArmSubsystem.getInstance().setHasCone(false);
      }
     break;
    }
    case OPEN:
    {
      open();
      if(newRobotState != null)
        newRobotState.accept(RobotGamePieceState.NoGamePiece);
      ArmSubsystem.getInstance().setHasCone(false);
      break;
    }
    case PICK_UP_CONE:
    {
      takeCone();
      if (newRobotState != null)
        newRobotState.accept(RobotGamePieceState.HasCone);
      ArmSubsystem.getInstance().setHasCone(true);
      break;
    }
    case PICK_UP_CUBE:
    {
      takeCube();
      if ( newRobotState != null)
        newRobotState.accept(RobotGamePieceState.HasCube);
      ArmSubsystem.getInstance().setHasCone(false);
      break;
    }
    default:
    {
      open();
      if(newRobotState != null)
        newRobotState.accept(RobotGamePieceState.NoGamePiece);
      ArmSubsystem.getInstance().setHasCone(false);
      break;
     }
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