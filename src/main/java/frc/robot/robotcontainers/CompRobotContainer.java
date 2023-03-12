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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.AlignToTargetVision;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandSelector;
import frc.robot.commands.ArmFFTestCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.GripperCommand.GRIPPER_INSTRUCTION;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RotateAngleXY;
import frc.robot.commands.RotateXYSupplier;
import frc.robot.commands.SetAngleArm;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TranslationCommand;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmPosition;
import frc.robot.config.ArmConfig.ArmSetpoint;
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
  private static RobotGamePieceState m_robotState = RobotGamePieceState.NoGamePiece;
  AutoSelector m_autoSelector;
  AutoRoutines routines;

    // getState and setState for commands managing the RobotGamePieceState
   public static Supplier<RobotGamePieceState> getState = CompRobotContainer::getRobotGamePieceState;
   public static Consumer<RobotGamePieceState> setState = a ->setRobotGamePieceState(a);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public CompRobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autoSelector = new AutoSelector();
    routines = new AutoRoutines();
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

    // Driver joystick
    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver));

    driver.start().onTrue(new ResetGyroToNearest());
    driver.back().onTrue(new ResetGyro());
    driver.b().onTrue(new ResetOdometry(new Pose2d()));

    driver.y().whileTrue(new RotateAngleXY(driver, 0));
    driver.a().whileTrue(new RotateAngleXY(driver, Math.PI));
    
    driver.x().whileTrue(new TranslationCommand(1, 1));
    driver.leftTrigger().whileTrue(new RotateXYSupplier(driver,
      NetworkTableInstance.getDefault().getTable("MergeVisionPipelineIntake22").getDoubleTopic("Yaw").subscribe(-99)
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
    operator.rightBumper().onTrue(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, setState));
    operator.back().onTrue(new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CUBE, setState).andThen(new WaitCommand(0.2)).andThen(new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP)));
    operator.start().onTrue(new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CONE, setState).andThen(new WaitCommand(0.4)).andThen(new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP)));
    
    operator.x().onTrue(new ArmCommand(ArmSetpoint.PICKUP));

    // Force the buttons to just do cone setpoints
    operator.a().onTrue(new ArmCommand(ArmSetpoint.BOTTOM_CONE));
    operator.b().onTrue(new ArmCommand(ArmSetpoint.MIDDLE_CONE));
    operator.y().onTrue(new ArmCommand(ArmSetpoint.TOP_CONE));

    // Choose the ArmSetpoint based on RobotGamePieceState
    // operator.a().onTrue(new ArmCommandSelector(getState, ArmPosition.GAME_PIECE_BOTTOM, false));
    // operator.b().onTrue(new ArmCommandSelector(getState, ArmPosition.GAME_PIECE_MIDDLE, false));
    // operator.y().onTrue(new ArmCommandSelector(getState, ArmPosition.GAME_PIECE_TOP, false));

    operator.rightTrigger().onTrue(new ArmCommand(ArmSetpoint.HUMAN_PLAYER_PICKUP));
    operator.leftTrigger().onTrue(new ArmCommand(ArmSetpoint.PICKUP_OUTSIDE_FRAME));

  

    // boolean isTop = true;

    // armStick.a().onTrue(new SetAngleArm(85, false, false).alongWith(new SetAngleArm(20, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
    //           //  .andThen((new SetAngleArm(85, false, false)).alongWith(new SetAngleArm(20, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance()))));

    // armStick.b().onTrue(new SetAngleArm(60, false, false).alongWith(new SetAngleArm(55, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
    // armStick.y().onTrue(new SetAngleArm(70, false, false).alongWith(new SetAngleArm(105, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
    // armStick.x().onTrue(new SetAngleArm(45, false, false).alongWith(new SetAngleArm(180, false, true)).alongWith(Commands.runOnce(()-> ArmSubsystem.getInstance(), ArmSubsystem.getInstance())));
    



    armStick.a().onTrue(new SetAngleArm(30, false, true));
    armStick.b().onTrue(new SetAngleArm(60, false, true));
    armStick.y().onTrue(new SetAngleArm(90, false, true));
    armStick.x().onTrue(new SetAngleArm(120, false, true));


    armStick.rightTrigger().toggleOnTrue(Commands.startEnd(
      () -> ArmSubsystem.getInstance().controlBottomArmBrake(true), 
      () -> ArmSubsystem.getInstance().controlBottomArmBrake(false)));

    armStick.leftTrigger().toggleOnTrue(Commands.startEnd(
      () -> ArmSubsystem.getInstance().controlTopArmBrake(true), 
      () -> ArmSubsystem.getInstance().controlTopArmBrake(false)));

    Command armFF = new ArmFFTestCommand(armStick, 7, false, true);

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
    int autoId = m_autoSelector.getAutoId();
    System.out.println("*********************** Auto Id"+autoId);
     return routines.getAutonomousCommand(autoId);
  }
  public static RobotGamePieceState getRobotGamePieceState() {
    return m_robotState;
  }
  public static void setRobotGamePieceState(RobotGamePieceState state) {
    m_robotState=state;
  }
}
