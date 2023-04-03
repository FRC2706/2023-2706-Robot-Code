// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

//This command is specifically for passing through the charge station 
//and then come back and land on the charge station

public class ChargeCommandPigeonExtend extends CommandBase {
  Timer m_timer = new Timer();

  ProfiledPIDController pidControlTheta;
  double currentTheta;
  double desiredTheta;

  double pigeonValue;
  double initPigeonValue;

  double initXPos;
  double currentX;
  double xLimit;

  double PITCH_TOLERANCE = 9;
  double X_WIDE_LIMIT = 7.0;
  // Unit: seconds
  double TIME_FOR_REVERSING = 0.18;
  double TIME_FOR_STATE4    = 1.0;
 
  int stateIndex = 0;
  /**
   * state 0: on the ground
   * state 1: climbing on the ramp
   * state 2: on the charge station
   * state 3: down the ramp
   * state 4: on the ground ( a timer)
   * state 5: change the x direction, on the ground
   * state 6: climing on the ramp with time out
   * state 7: very slow speed on the charge station
   * state 8: stop in the middle of the charge station (a timer)
   */
  // Unit: meters per second
  //states              0,  1,   2,    3,   4,   5,     6,    7,   8
  double[] xSpeeds = {1.5, 1.0, 1.5, 1.5, 1.5, -1.5, -1.5, -0.2, 0.5 };
  //double[] xSpeeds = {1.5, 0.5, 1.0, 1.0, 1.0, -1.5, -1.5, -0.2, 0.5 };

  BlingSubsystem bling;  
  
  /** Creates a new ChargeCommandPigeonExtend. */
  public ChargeCommandPigeonExtend() {
    pidControlTheta = new ProfiledPIDController(5.0,0, 0.4, new Constraints(4*Math.PI, 8*Math.PI));
    pidControlTheta.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());

    bling = BlingSubsystem.getINSTANCE();
    if (bling != null) {
      addRequirements(bling);
    }
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    //keep the current angle
    desiredTheta = currentTheta; 

    //reset current positions
    pidControlTheta.reset(currentTheta);

    //wide side to go to the charge station
    //use the roll value
    initPigeonValue = SwerveSubsystem.getInstance().getRoll();

    xLimit = X_WIDE_LIMIT;
    initXPos = SwerveSubsystem.getInstance().getPose().getX();

    stateIndex = 0;

    bling.setDisabled();

    m_timer.stop();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    double theta = pidControlTheta.calculate(currentTheta, desiredTheta);

    currentX = SwerveSubsystem.getInstance().getPose().getX();
    //wide side
    pigeonValue = SwerveSubsystem.getInstance().getRoll();

    //update the state
    if ((Math.abs(initPigeonValue - pigeonValue) > PITCH_TOLERANCE +2)
        && (( stateIndex == 0) || (stateIndex == 2) || (stateIndex == 5) ))
    {
      stateIndex = stateIndex + 1;
      System.out.println("============ChargeCmd state " + stateIndex);

      if ( stateIndex == 6 )
      {
        m_timer.reset();
        m_timer.restart();
      }
    }
    else if ((Math.abs(initPigeonValue - pigeonValue) < 9)
            && ( (stateIndex == 1) || (stateIndex == 3) || (stateIndex == 7)))
    {
      stateIndex = stateIndex + 1;
      System.out.println("============ChargeCmd state " + stateIndex);

      if( stateIndex == 2)
      {
        bling.setRed();
      }

      if( stateIndex == 4 )
      {
        //let timer to decide when go to state 5
        m_timer.reset();
        m_timer.restart();
        bling.setBlue();
      }   

      if( stateIndex == 8 )
      {
       //isFinished will terminate the command
       m_timer.reset();
        m_timer.restart();
        bling.setRed();
      }

    }  

    if(stateIndex == 6 &&  m_timer.hasElapsed(0.4))
    {
      stateIndex = stateIndex + 1;
      System.out.println("============ChargeCmd state " + stateIndex);
      // m_timer.stop();
      // m_timer.reset();
    }

    if ( (stateIndex == 4 ) && (m_timer.hasElapsed(TIME_FOR_STATE4)))
    {
      stateIndex = stateIndex + 1;
      System.out.println("============ChargeCmd state " + stateIndex);
      // m_timer.stop();
      // m_timer.reset();
    }
    
    SwerveSubsystem.getInstance().drive(xSpeeds[stateIndex], 0, theta, true, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if ((stateIndex == 8 && m_timer.hasElapsed(TIME_FOR_REVERSING)) || Math.abs(initXPos - currentX) > xLimit)
    return (true);
  else
    return (false);
  }
}
