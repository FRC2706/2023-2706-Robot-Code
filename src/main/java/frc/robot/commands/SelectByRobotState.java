// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotcontainers.CompRobotContainer.RobotState;

public class SelectByRobotState extends CommandBase {
  /** Creates a new SelectByRobotState. */
  Supplier<RobotState> m_getState;
  Command m_noGamePiece;
  Command m_cone;
  Command m_cube;
  public SelectByRobotState(Supplier<RobotState> getState,Command noGamePiece, Command cone, Command cube) {
    m_getState=getState;
    m_noGamePiece= noGamePiece;
    m_cone= cone;
    m_cube= cube;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState state = m_getState.get();
    if (state==RobotState.NoGamePiece) {
      m_noGamePiece.schedule();
    }
    else if (state==RobotState.Cone) {
      m_cone.schedule();
    }
    else if (state==RobotState.Cube) {
      m_cube.schedule();
    }
    }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
