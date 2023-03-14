// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeCommand extends CommandBase {
  ProfiledPIDController pidControlX;
  double currentX;
  double desiredX;
  double deltaX;

  ProfiledPIDController pidControlTheta;
  double currentTheta;
  double desiredTheta;

  double rollValue;
  double initRollValue;

  boolean hasReachedAngle;

  double ROLL_TOLERANCE = 15;
  /** Creates a new ChargeCommand. */
  public ChargeCommand( double deltaX) {
    pidControlX = new ProfiledPIDController(1, 0.0, 0.2, 
    new TrapezoidProfile.Constraints(1, 1));

    pidControlTheta = new ProfiledPIDController(5.0,0, 0.4,
    new TrapezoidProfile.Constraints(4*Math.PI, 8*Math.PI));

    this.deltaX = deltaX;
    // Use addRequirements() here to declare subsystem dependencies.
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
    initRollValue = SwerveSubsystem.getInstance().getRollValue();

    hasReachedAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = SwerveSubsystem.getInstance().getPose().getX();
    double x = pidControlX.calculate(currentX, desiredX);

    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    double theta = pidControlTheta.calculate(currentTheta, desiredTheta);

    SwerveSubsystem.getInstance().drive(x, 0, theta, true, false);

    rollValue = SwerveSubsystem.getInstance().getRollValue();

    if (hasReachedAngle == false && Math.abs(initRollValue - rollValue) > ROLL_TOLERANCE +3) 
    {
      hasReachedAngle = true;
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
    if (hasReachedAngle && Math.abs(initRollValue - rollValue) < ROLL_TOLERANCE )
      return (true);
    else
      return (false);
  }
}
