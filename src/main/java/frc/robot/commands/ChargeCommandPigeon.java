// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeCommandPigeon extends CommandBase {
  Timer m_timer = new Timer();

  ProfiledPIDController pidControlTheta;
  double currentTheta;
  double desiredTheta;

  double pigeonValue;
  double initPigeonValue;
  boolean bWideSide;
  double flipXDirection;

  double PITCH_TOLERANCE = 10;

  //@todo: tune these values
  double TIME_FOR_REVERSING = 0.10;
  double X_SPEED_STATE0 = 1.5;
  double X_SPEED_STATE1 = 1.0;
  double X_SPEED_STATE2 = 0.5;

  int state = 0;
  double flipXState0;
  double flipXState1;
  double flipXState2;

  /** Creates a new ChargeCommand. */
  public ChargeCommandPigeon( boolean bWideSide, double flipXDirection) {
    pidControlTheta = new ProfiledPIDController(5.0,0, 0.4,
          new Constraints(4*Math.PI, 8*Math.PI));

    this.bWideSide = bWideSide;
    this.flipXDirection = flipXDirection;
    pidControlTheta.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    //keep the current angle
    desiredTheta = currentTheta;    

    //reset current positions
    pidControlTheta.reset(currentTheta);

    if( bWideSide == true)
    {
      //use the roll value
      initPigeonValue = SwerveSubsystem.getInstance().getRoll();

    }
    else
    {
      //use the pitch value
      initPigeonValue = SwerveSubsystem.getInstance().getPitch();
    }

    state = 0;

    flipXState0 = flipXDirection;
    flipXState1 = flipXDirection;
    flipXState2 = - flipXDirection;

    m_timer.stop();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    double theta = pidControlTheta.calculate(currentTheta, desiredTheta);

    if( bWideSide == true )
    {
      pigeonValue = SwerveSubsystem.getInstance().getRoll();
    }
    else
    {
      pigeonValue = SwerveSubsystem.getInstance().getPitch();
    }
    

    if (state == 0 && Math.abs(initPigeonValue - pigeonValue) > PITCH_TOLERANCE +1) 
    {
      state = 1;
    }
    if (state == 1 && Math.abs(initPigeonValue - pigeonValue) < PITCH_TOLERANCE) 
    {
      state = 2;
      m_timer.restart();
    }

    double xSpeed = 0.0;
    double xFlip = 1.0;

    switch (state)
    {
      case 0:
        xSpeed = X_SPEED_STATE0;
        xFlip = flipXState0;
        break;
      case 1:
        xSpeed = X_SPEED_STATE1;
        xFlip = flipXState1;
        break;
      case 2:
        xSpeed = X_SPEED_STATE2;
        xFlip = flipXState2;
        break;
      default:
      break;
    }

    SwerveSubsystem.getInstance().drive(xSpeed*xFlip, 0, theta, true, false);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == 2 && m_timer.hasElapsed(TIME_FOR_REVERSING))
      return (true);
    else
      return (false);
  }
}
