// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeCommand extends CommandBase {
  Timer m_timer = new Timer();
  ProfiledPIDController pidControlX;
  double currentX;
  double desiredX;
  double deltaX;

  ProfiledPIDController pidControlTheta;
  double currentTheta;
  double desiredTheta;

  double pitchValue;
  double initPitchValue;

  int state = 0;

  double PITCH_TOLERANCE = 10;
  double TIME_FOR_REVERSING = 0.10;
  /** Creates a new ChargeCommand. */
  public ChargeCommand( double deltaX) {
    pidControlX = new ProfiledPIDController(0.8, 0.0, 0.2, 
          new Constraints(0.8, 0.8));

    pidControlTheta = new ProfiledPIDController(5.0,0, 0.4,
          new Constraints(4*Math.PI, 8*Math.PI));

    this.deltaX = deltaX;

    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentX = SwerveSubsystem.getInstance().getPose().getX();
    desiredX = currentX + deltaX;

    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    //keep the current angle
    desiredTheta = currentTheta;    

    //reset current positions
    pidControlX.reset(currentX);
    pidControlTheta.reset(currentTheta);

    //get the initial roll value
    initPitchValue = SwerveSubsystem.getInstance().getPitch();

    state = 0;
    m_timer.stop();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = SwerveSubsystem.getInstance().getPose().getX();
    double x = pidControlX.calculate(currentX, desiredX);

    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    double theta = pidControlTheta.calculate(currentTheta, desiredTheta);


    pitchValue = SwerveSubsystem.getInstance().getPitch();

    if (state == 0 && Math.abs(initPitchValue - pitchValue) > PITCH_TOLERANCE +1) 
    {
      state = 1;
    }
    if (state == 1 && Math.abs(initPitchValue - SwerveSubsystem.getInstance().getPitch()) < PITCH_TOLERANCE) {
      state = 2;
      m_timer.restart();
    }

    if (state == 2) {
      SwerveSubsystem.getInstance().drive(-1 * Math.copySign(0.25, deltaX), 0, 0, true, false);
    } else {
      SwerveSubsystem.getInstance().drive(x, 0, theta, true, false);
    }


    //@todo: use the deltaRollValue to determine x.

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
