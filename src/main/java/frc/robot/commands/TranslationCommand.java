// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class TranslationCommand extends CommandBase {
  ProfiledPIDController pidControlX;
  double currentX;
  double desiredX;
  double deltaX;

  double TOLERANCE = 0.05;

  ProfiledPIDController pidControlY;
  double currentY;
  double desiredY;
  double deltaY;

  ProfiledPIDController pidControlTheta;
  double currentTheta;
  double desiredTheta;
  
  /** Creates a new translation. */
  public TranslationCommand( double deltaX, double deltaY) {
    pidControlX = new ProfiledPIDController(1, 0.0, 0.2, 
                                           new TrapezoidProfile.Constraints(1, 1));
    pidControlY = new ProfiledPIDController(1, 0.0, 0.2, 
                                           new TrapezoidProfile.Constraints(1,1));
    pidControlTheta = new ProfiledPIDController(5.0,0, 0.4,
                                           new TrapezoidProfile.Constraints(4*Math.PI, 8*Math.PI));

    this.deltaX = deltaX;
    this.deltaY = deltaY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //from odometry get currentX and currentY
    currentX = SwerveSubsystem.getInstance().getPose().getX();
    currentY = SwerveSubsystem.getInstance().getPose().getY();
    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();

    desiredX = currentX + deltaX;
    desiredY = currentY + deltaY;
    //keep the current angle
    desiredTheta = currentTheta; 

    //reset current positions
    pidControlX.reset(currentX);
    pidControlY.reset(currentY);
    pidControlTheta.reset(currentTheta);

    //set the tolerance
    pidControlX.setTolerance(TOLERANCE, TOLERANCE);
    pidControlY.setTolerance(TOLERANCE, TOLERANCE);
    pidControlTheta.setTolerance(TOLERANCE, TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Current X: " + currentX + ", DesiredX: " + desiredX + ", Position Error: " + pidControlX.getPositionError());
    System.out.println("Current Y: " + currentY + ", DesiredY: " + desiredY + ", Position Error: " + pidControlY.getPositionError());
    //update the currentX and currentY
    currentX = SwerveSubsystem.getInstance().getPose().getX();
    currentY = SwerveSubsystem.getInstance().getPose().getY();
    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();

    double x = pidControlX.calculate(currentX, desiredX);
    double y = pidControlY.calculate(currentY, desiredY);
    double theta = pidControlTheta.calculate(currentTheta, desiredTheta);

    SwerveSubsystem.getInstance().drive(x, y, theta, true, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
    //todo: set the wheel angles
    //todo: set brake mode
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(currentX - desiredX) < TOLERANCE 
    && Math.abs(currentY - desiredY) < TOLERANCE
    && Math.abs(currentTheta - desiredTheta) < TOLERANCE)
    {
      return(true);
    }
    else {
      return(false);
    }
    //@todo: testing
    /*if(pidControlX.atSetpoint() && pidControlY.atSetpoint())
    {
      return true;
    }
    else
    {
      return false;
    }*/
  }
}
