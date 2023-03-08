// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import java.util.function.Consumer;
import java.util.function.Supplier;

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
import frc.robot.commands.ArmCommandExample;
import frc.robot.commands.ArmFFTestCommand;
import frc.robot.commands.IntakeCommand;
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
    // CommandXboxController
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);
    CommandXboxController testStick = new CommandXboxController(2);
    CommandXboxController armStick = new CommandXboxController(3);

    // getState and setState for commands managing the RobotGamePieceState
    Supplier<RobotGamePieceState> getState = ()-> getRobotGamePieceState();
    Consumer<RobotGamePieceState> setState = a ->setRobotGamePieceState(a);

    SwerveModuleState state1 =  new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    SwerveModuleState state2 =  new SwerveModuleState(0, Rotation2d.fromDegrees(90));
    SwerveModuleState state3 =  new SwerveModuleState(-1.5, Rotation2d.fromDegrees(0));
    SwerveModuleState state4 =  new SwerveModuleState(1.5, Rotation2d.fromDegrees(0));

    // Driver joystick
    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver));

    driver.start().onTrue(new ResetGyroToNearest());
    driver.back().onTrue(new ResetGyro());
    driver.b().onTrue(new ResetOdometry(new Pose2d()));

    driver.y().whileTrue(new RotateAngleXY(driver, 0));
    driver.a().whileTrue(new RotateAngleXY(driver, Math.PI));
    
    driver.x().whileTrue(new TranslationCommand(1, 1));
    driver.leftTrigger().whileTrue(new RotateXYSupplier(driver,
      NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99)
    ));

    driver.rightTrigger().whileTrue(Commands.sequence(
      new AlignToTargetVision(driver,
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99),
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("DistanceToTarget").subscribe(-99),
        2.0,
        0.3,
        1.5,
        1.5),
      new AlignToTargetVision(driver,
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99),
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("DistanceToTarget").subscribe(-99),
        1.5,
        0.03,
        1,
        0.5)
    ));


    // Operator Joystick
    operator.rightBumper().whileTrue(new IntakeCommand(1, setState));
    operator.back().whileTrue(new IntakeCommand(3, setState));
    operator.start().whileTrue(new IntakeCommand(2, setState));


    
    Command angleSetPoint1 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state1, state1, state1, state1}, true), SwerveSubsystem.getInstance());
    testStick.a().whileTrue(angleSetPoint1).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));

    Command angleSetPoint2 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state2, state2, state2, state2}, true), SwerveSubsystem.getInstance());
    testStick.b().whileTrue(angleSetPoint2).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));

    Command speedSetPoint1 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state3, state3, state3, state3}, true), SwerveSubsystem.getInstance());
    testStick.y().whileTrue(speedSetPoint1).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));
    
    Command speedSetPoint2 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state4, state4, state4, state4}, true), SwerveSubsystem.getInstance());
    testStick.x().whileTrue(speedSetPoint2).whileFalse(new InstantCommand(SwerveSubsystem.getInstance()::stopMotors, SwerveSubsystem.getInstance()));
  

    boolean isTop = true;

    armStick.a().onTrue(new SetAngleArm(85, false, false).alongWith(new SetAngleArm(20, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
              //  .andThen((new SetAngleArm(85, false, false)).alongWith(new SetAngleArm(20, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance()))));

    armStick.b().onTrue(new SetAngleArm(60, false, false).alongWith(new SetAngleArm(55, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
    armStick.y().onTrue(new SetAngleArm(70, false, false).alongWith(new SetAngleArm(105, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
    armStick.x().onTrue(new SetAngleArm(45, false, false).alongWith(new SetAngleArm(180, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));


    armStick.rightTrigger().toggleOnTrue(Commands.startEnd(
      () -> ArmSubsystem.getInstance().controlBottomArmBrake(true), 
      () -> ArmSubsystem.getInstance().controlBottomArmBrake(false)));

    armStick.leftTrigger().toggleOnTrue(Commands.startEnd(
      () -> ArmSubsystem.getInstance().controlTopArmBrake(true), 
      () -> ArmSubsystem.getInstance().controlTopArmBrake(false)));

    Command armFF = new ArmFFTestCommand(armStick, 7, true, true);

    armStick.leftBumper().onTrue(Commands.runOnce(() -> armFF.schedule()));
    armStick.rightBumper().onTrue(Commands.runOnce(() -> ArmSubsystem.getInstance().updatePIDSettings()));
    armStick.back().onTrue(Commands.sequence(
            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()),
            Commands.runOnce(() -> ArmSubsystem.getInstance().stopMotors())
        ));
    // armStick.start().onTrue(Commands.runOnce(() -> ArmSubsystem.getInstance().resetEncoder(100, Math.toRadians(-90)))); // 100 is an arbitrary number
    armStick.start().onTrue(Commands.runOnce(() -> ArmSubsystem.getInstance().updateFromAbsoluteTop()));

    // Construct the RelaySubsystem so the NTRelays are constructed
    RelaySubsystem.getInstance();
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
