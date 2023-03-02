// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmFFTestCommand;
import frc.robot.commands.SetAngleArm;
import frc.robot.config.ArmConfig;
import frc.robot.subsystems.ArmDisplay;
import frc.robot.subsystems.ArmSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class ArmBotContainer extends RobotContainer{


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public ArmBotContainer() {
    // Configure the button bindings
    LiveWindow.enableAllTelemetry();

    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController controlStick = new CommandXboxController(1);

    //controlStick.y().onTrue(new ArmCommand(3, true));
    //controlStick.b().onTrue(new ArmCommand(2, true));
    //controlStick.a().onTrue(new ArmCommand(1, true));
    //controlStick.x().onTrue(new ArmCommand(0, true));

    controlStick.b().onTrue(new SetAngleArm(-1 * Math.PI/2, false));
    controlStick.y().onTrue(new SetAngleArm(0, false));
    controlStick.a().onTrue(new SetAngleArm(Math.PI/8, false));
    controlStick.start().onTrue(Commands.runOnce(() -> ArmSubsystem.getInstance().resetEncoder()));

    Command topArmFF = new ArmFFTestCommand(controlStick, 2);

    controlStick.leftBumper().onTrue(Commands.runOnce(() -> topArmFF.schedule()));
    controlStick.back().onTrue(Commands.sequence(
            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()),
            Commands.runOnce(() -> ArmSubsystem.getInstance().stopMotors())
        ));

    controlStick.rightBumper().onTrue(Commands.runOnce(() -> ArmSubsystem.getInstance().updatePIDSettings()));


    ArmDisplay display = new ArmDisplay(ArmConfig.L1, ArmConfig.L2);
    driver.a().onTrue(new ArmCommand(ArmConfig.ArmSetpoint.LOW, false));
    driver.b().onTrue(Commands.runOnce(() -> display.updateSetpointDisplay(Math.toRadians(45), Math.toRadians(90))));
    driver.x().onTrue(Commands.runOnce(() -> display.updateSetpointDisplay(Math.toRadians(45), Math.toRadians(135))));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return null;
  }


}
