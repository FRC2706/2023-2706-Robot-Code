// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.AlignToTargetVision;
import frc.robot.commands.ArmFFTestCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RotateAngleXY;
import frc.robot.commands.RotateXYSupplier;
import frc.robot.commands.SetAngleArm;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TranslationCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RelaySubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class CompRobotContainer extends RobotContainer {

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
    CommandXboxController driverStick = new CommandXboxController(0);
    CommandXboxController controlStick = new CommandXboxController(1);
    CommandXboxController testStick = new CommandXboxController(2);

    SwerveModuleState state1 =  new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    SwerveModuleState state2 =  new SwerveModuleState(0, Rotation2d.fromDegrees(90));
    SwerveModuleState state3 =  new SwerveModuleState(-1.5, Rotation2d.fromDegrees(0));
    SwerveModuleState state4 =  new SwerveModuleState(1.5, Rotation2d.fromDegrees(0));

    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driverStick));

    driverStick.start().onTrue(new ResetGyroToNearest());
    driverStick.back().onTrue(new ResetGyro());
    driverStick.b().onTrue(new ResetOdometry(new Pose2d()));

    driverStick.y().whileTrue(new RotateAngleXY(driverStick, 0));
    driverStick.a().whileTrue(new RotateAngleXY(driverStick, Math.PI));
    
    driverStick.x().whileTrue(new TranslationCommand(1, 1));
    driverStick.leftTrigger().whileTrue(new RotateXYSupplier(driverStick,
      NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99)
    ));

    driverStick.rightTrigger().whileTrue(Commands.sequence(
      new AlignToTargetVision(driverStick,
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99),
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("DistanceToTarget").subscribe(-99),
        2.0,
        0.3,
        1.5,
        1.5),
      new AlignToTargetVision(driverStick,
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99),
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("DistanceToTarget").subscribe(-99),
        1.5,
        0.03,
        1,
        0.5)
    ));

    RelaySubsystem.getInstance().setRelay(Config.RELAY_RINGLIGHT_REAR_LARGE, true);

    
    Command angleSetPoint1 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state1, state1, state1, state1}, true), SwerveSubsystem.getInstance());
    testStick.a().whileTrue(angleSetPoint1).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));

    Command angleSetPoint2 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state2, state2, state2, state2}, true), SwerveSubsystem.getInstance());
    testStick.b().whileTrue(angleSetPoint2).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));

    Command speedSetPoint1 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state3, state3, state3, state3}, true), SwerveSubsystem.getInstance());
    testStick.y().whileTrue(speedSetPoint1).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));
    
    Command speedSetPoint2 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state4, state4, state4, state4}, true), SwerveSubsystem.getInstance());
    testStick.x().whileTrue(speedSetPoint2).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));
  

    boolean isTop = true;
    CommandXboxController armStick = new CommandXboxController(3);

    armStick.a().onTrue(new SetAngleArm(30, false, isTop));
    armStick.b().onTrue(new SetAngleArm(60, false, isTop));
    armStick.y().onTrue(new SetAngleArm(90, false, isTop));
    armStick.x().onTrue(new SetAngleArm(120, false, isTop));

    Command armFF = new ArmFFTestCommand(armStick, 2);

    armStick.leftBumper().onTrue(Commands.runOnce(() -> armFF.schedule()));
    armStick.back().onTrue(Commands.sequence(
            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()),
            Commands.runOnce(() -> ArmSubsystem.getInstance().stopMotors())
        ));
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
}
