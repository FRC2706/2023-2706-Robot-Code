// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.commands.BrakeModeDisabled;
import frc.robot.commands.CheckArmSetpoints;
import frc.robot.commands.SetBlingCommand;
import frc.robot.commands.SyncArmEncoders;
import frc.robot.commands.SyncArmEncodersV2;
import frc.robot.commands.SyncSteerEncoders;
import frc.robot.config.Config;
import frc.robot.robotcontainers.ArmBotContainer;
import frc.robot.robotcontainers.BeetleContainer;
import frc.robot.robotcontainers.ClutchContainer;
import frc.robot.robotcontainers.CompRobotContainer;
import frc.robot.robotcontainers.MergonautContainer;
import frc.robot.robotcontainers.MiniNeoDiffContainer;
import frc.robot.robotcontainers.MiniSwerveContainer;
import frc.robot.robotcontainers.RobotContainer;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.DiffNeoSubsystem;
import frc.robot.subsystems.DiffTalonSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private BrakeModeDisabled brakeModeDisabledCommand = null;

  Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

  private boolean m_firstDisableAtBootup = true;




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
    pcmCompressor.enableDigital();
    PathPlannerServer.startServer(5811);
    // Instantiate the RobotContainer based on the Robot ID.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    switch (Config.getRobotId()) {
      case 0:
        m_robotContainer = new CompRobotContainer(); break;
      
      case 1:
        m_robotContainer = new ClutchContainer(); break;

      case 2:
        m_robotContainer = new BeetleContainer(); break;

      case 3:
        m_robotContainer = new MergonautContainer(); break;

      case 4:
        m_robotContainer = new MiniSwerveContainer(); break;

      case 5:
        m_robotContainer = new MiniNeoDiffContainer(); break;
      
      case 6:
        m_robotContainer = new ArmBotContainer(); break;

      default:
        m_robotContainer = new CompRobotContainer();
        DriverStation.reportError(
            String.format("ISSUE WITH CONSTRUCTING THE ROBOT CONTAINER. \n " +
                          "CompRobotContainer constructed by default. RobotID: %d", Config.getRobotId()), 
            true);
    }

    if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffNeoSubsystem) ||
        SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffTalonSubsystem)) 
    {
      brakeModeDisabledCommand = new BrakeModeDisabled();
    } 
    
    if (SubsystemChecker.canSubsystemConstruct(SubsystemType.SwerveSubsystem)) {
      BlingSubsystem.getINSTANCE().setRed();
      new SyncSteerEncoders().schedule();
      new SyncArmEncodersV2().schedule();
    } 

    // Add CommandScheduler to shuffleboard so we can display what commands are scheduled
    ShuffleboardTab basicDebuggingTab = Shuffleboard.getTab("BasicDebugging");
    basicDebuggingTab
      .add("CommandScheduler", CommandScheduler.getInstance())
      .withPosition(3, 0)
      .withSize(3, 6);

      
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (brakeModeDisabledCommand != null) 
      brakeModeDisabledCommand.schedule(); 
      
    if (SubsystemChecker.canSubsystemConstruct(SubsystemType.BlingSubsystem)) {
      if (m_firstDisableAtBootup) {
        m_firstDisableAtBootup = false;
      } else {
        BlingSubsystem.getINSTANCE().setDisabled();
      }
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link CompRobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    // Set the IdleMode or NeutralMode of Differential subsystem
    if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffNeoSubsystem)) {
      DiffNeoSubsystem.getInstance().setIdleMode(Config.DIFF.AUTO_IDLEMODE);

    } else if (SubsystemChecker.canSubsystemConstruct(SubsystemType.DiffTalonSubsystem)) {
      DiffTalonSubsystem.getInstance().setNeutralMode(Config.DIFF.AUTO_NEUTRALMODE);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static void haltRobot(String msg, Exception e) {

  }
}
