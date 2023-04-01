// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ArmConfig.ArmPosition;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommandSelector extends CommandBase {
  Supplier<RobotGamePieceState> m_robotState;
  ArmPosition m_Position;
  ArmSetpoint m_ArmSetPoint;
  boolean m_slowerAcceleration;
  Command m_ArmCommand;
  CommandXboxController m_operator_stick;
  
  /** Creates a new ArmCommand. */
  public ArmCommandSelector(Supplier<RobotGamePieceState> robotState, 
                            ArmPosition position,
                            boolean slowerAcceleration, CommandXboxController operator_stick) {
    //get the current robotState: from container
    m_robotState = robotState;
    m_Position = position;
    m_slowerAcceleration = slowerAcceleration;
    m_operator_stick = operator_stick;

    addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get the current robotState: from container
    RobotGamePieceState robotState = m_robotState.get();
    switch (robotState) {
      case NoGamePiece: 
        m_ArmSetPoint = ArmSetpoint.PICKUP;
      break;
      case HasNoseCone:
      case HasBaseCone:
      //todo: the arm's speed needs to be changed to suit these two cases
        if (m_Position == ArmPosition.GAME_PIECE_BOTTOM) {
          m_ArmSetPoint = ArmSetpoint.BOTTOM_CONE;
        }
        else if (m_Position == ArmPosition.GAME_PIECE_MIDDLE) {
          m_ArmSetPoint = ArmSetpoint.MIDDLE_CONE;
        }
        else if (m_Position == ArmPosition.GAME_PIECE_TOP) {
          m_ArmSetPoint = ArmSetpoint.TOP_CONE;
        }
        else {
          m_ArmSetPoint = ArmSetpoint.HOME_WITH_GAMEPIECE;
        }
      break;
      case HasCube:
        if (m_Position == ArmPosition.GAME_PIECE_BOTTOM) {
          m_ArmSetPoint = ArmSetpoint.BOTTOM_CUBE;
        }
        else if (m_Position == ArmPosition.GAME_PIECE_MIDDLE) {
          m_ArmSetPoint = ArmSetpoint.MIDDLE_CUBE;
        }
        else if (m_Position == ArmPosition.GAME_PIECE_TOP) {
          m_ArmSetPoint = ArmSetpoint.TOP_CUBE;
        }
        else {
          m_ArmSetPoint = ArmSetpoint.HOME_WITH_GAMEPIECE;
        }
      break;
      default: 
        m_ArmSetPoint = ArmSetpoint.HOME_WITH_GAMEPIECE;
      break;
    }
    if (m_ArmSetPoint == ArmSetpoint.MIDDLE_CONE || m_ArmSetPoint == ArmSetpoint.TOP_CONE) {
      m_ArmCommand = new ArmJoystickConeCommand (m_ArmSetPoint, m_operator_stick);
    }
    else {
      m_ArmCommand = new ArmCommand (m_ArmSetPoint);
    }
    m_ArmCommand.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ArmCommand.isFinished();
  }
}
