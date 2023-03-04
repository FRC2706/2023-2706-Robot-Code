// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.commands.ArmCommandExample;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class CompRobotContainer extends RobotContainer {
  public enum RobotGamePieceState {
    NoGamePiece,
    HasCone,
    HasCube
  }
  private RobotGamePieceState m_robotState = RobotGamePieceState.NoGamePiece;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public CompRobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Joystick driverStick = new Joystick(0);
    Joystick controlStick = new Joystick(1);

    Supplier<RobotGamePieceState> getState = ()-> getRobotGamePieceState();
    Consumer<RobotGamePieceState> setState = a ->setRobotGamePieceState(a);

    //examples 
    ArmCommandExample armTomCmd = new ArmCommandExample (getState,  0);
    ArmCommandExample armMiddleCmd = new ArmCommandExample(getState,  1);
    ArmCommandExample armBottomCmd = new ArmCommandExample(getState, 2);

    IntakeCommand intakeCmd = new IntakeCommand (0,setState);
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return new InstantCommand(); 
  }
  public RobotGamePieceState getRobotGamePieceState() {
    return m_robotState;
  }
  public void setRobotGamePieceState(RobotGamePieceState state) {
    m_robotState=state;
  }
}
