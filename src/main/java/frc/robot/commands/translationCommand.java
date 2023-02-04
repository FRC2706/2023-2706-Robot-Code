// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.html.HTMLDocument.RunElement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class translationCommand extends CommandBase {
  ProfiledPIDController pidControlX;
  double currentX;
  double desiredX;
  double deltaX;

  double TOLERANCE = 0.05;


  ProfiledPIDController pidControlY;
  double currentY;
  double desiredY;
  double deltaY;

  
  /** Creates a new translation. */
  public translationCommand( double deltaX, double deltaY) {
    pidControlX = new ProfiledPIDController(1, 0.0, 0.2, 
                                           new TrapezoidProfile.Constraints(1, 1));
    pidControlY = new ProfiledPIDController(1, 0.0, 0.2, 
                                           new TrapezoidProfile.Constraints(1,1));

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

    desiredX = currentX + deltaX;
    desiredY = currentY + deltaY;

    //set the tolerance
    pidControlX.setTolerance(TOLERANCE, TOLERANCE);
    pidControlY.setTolerance(TOLERANCE, TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Current X: " + currentX + ", DesiredX: " + desiredX + ", Position Error: " + pidControlX.getPositionError());
    System.out.println("Current Y: " + currentY + ", DesiredY: " + desiredY + ", Position Error: " + pidControlY.getPositionError());
    //update the currentX and currentY
    currentX = SwerveSubsystem.getInstance().getPose().getX();
    currentY = SwerveSubsystem.getInstance().getPose().getY();

    double x = pidControlX.calculate(currentX, desiredX);
    double y = pidControlY.calculate(currentY, desiredY);

    SwerveSubsystem.getInstance().drive(x, y, 0, true, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
    //todo: set brake mode
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(currentX - desiredX) < TOLERANCE && Math.abs(currentY - desiredY) < TOLERANCE) {
      return(true);
    }
    else {
      return(false);
    }
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
