// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfig.ArmPosition;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;

public class ArmCommandSelector extends CommandBase {
  Supplier<RobotGamePieceState> m_robotState;
  ArmPosition m_Position;
  ArmSetpoint m_ArmSetPoint;
  boolean m_slowerAcceleration;
  ArmCommand m_ArmCommand;
  
  /** Creates a new ArmCommand. */
  public ArmCommandSelector(Supplier<RobotGamePieceState> robotState, 
                            ArmPosition position,
                            boolean slowerAcceleration ) {
    //get the current robotState: from container
    m_robotState = robotState;
    m_Position = position;
    m_slowerAcceleration = slowerAcceleration;
    // Use addRequirements() here to declare subsystem dependencies.
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
      case HasCone:
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
    m_ArmCommand = new ArmCommand (m_ArmSetPoint);
    m_ArmCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
