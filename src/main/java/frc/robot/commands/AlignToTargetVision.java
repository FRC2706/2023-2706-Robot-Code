// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionNTSubsystem;

public class AlignToTargetVision extends CommandBase {

  ProfiledPIDController pidX = new ProfiledPIDController(4, 0.0, 0.2, 
                                          new TrapezoidProfile.Constraints(2, 2));
  ProfiledPIDController pidY = new ProfiledPIDController(4, 0.0, 0.2, 
                                          new TrapezoidProfile.Constraints(2, 2));
  ProfiledPIDController pidRot = new ProfiledPIDController(5.0, 0, 0.4, 
                                          new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI)); //pid to be tested

  double m_visionDistanceFromTargetY;
  double m_distFromTarget;

  boolean m_isTapeNotApril;

  double xSetpoint = 0;
  double ySetpoint = 0;
  double rotSetpoint = Math.PI;

  double m_tolerance = 0.05;

  Timer time = new Timer();

  boolean m_finishedWhenAligned;

  Timer m_timer = new Timer();

  boolean timerStarted = false;

  /** Creates a new AlignWithNode. */
  
  public AlignToTargetVision(boolean isTapeNotApril, double distFromTarget, double tolerance, double desiredVisionY, double desiredHeading, double vel, double accel, boolean finishedWhenAligned) {
    m_isTapeNotApril = isTapeNotApril;
    
    m_distFromTarget = distFromTarget;
    m_visionDistanceFromTargetY = desiredVisionY;
    m_tolerance = tolerance;

    rotSetpoint = desiredHeading;
    pidX.setConstraints(new Constraints(vel, accel));
    pidY.setConstraints(new Constraints(vel, accel));

    m_finishedWhenAligned = finishedWhenAligned;

    

    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidX.reset(SwerveSubsystem.getInstance().getPose().getX());
    pidY.reset(SwerveSubsystem.getInstance().getPose().getY());
    pidRot.reset(SwerveSubsystem.getInstance().getHeading().getRadians());

    //set the tolerance
    // pidX.setTolerance(0.25, 0.25);   //
    // pidY.setTolerance(0.25, 0.25);   // tolerance values TBD
    // pidRot.setTolerance(Math.PI / 4.0, Math.PI / 4.0);                    //

    pidRot.enableContinuousInput(-Math.PI, Math.PI);

    xSetpoint = SwerveSubsystem.getInstance().getPose().getX();
    ySetpoint = SwerveSubsystem.getInstance().getPose().getY();

    m_timer.restart();
    timerStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d visionTarget;
    if (m_isTapeNotApril) {
      visionTarget = VisionNTSubsystem.getInstance().calculateTapeTarget();
    } else {
      visionTarget = VisionNTSubsystem.getInstance().calculateAprilTarget();
    }
  
    if(visionTarget != null){
      xSetpoint =  visionTarget.getX() + m_distFromTarget;
      ySetpoint = visionTarget.getY() + m_visionDistanceFromTargetY;
    }

    double xSpeed = pidX.calculate(SwerveSubsystem.getInstance().getPose().getX(), xSetpoint);
    double ySpeed = pidY.calculate(SwerveSubsystem.getInstance().getPose().getY(), ySetpoint);
    double rot = pidRot.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), rotSetpoint);
    
    // System.out.println("X: " + SwerveSubsystem.getInstance().getPose().getX() + ", Setpoint: " + xSetpoint + ", Y: " + SwerveSubsystem.getInstance().getPose().getY() + ", Setpoint: " + ySetpoint + "Rot: " + SwerveSubsystem.getInstance().getHeading().getRadians() + ", Setpoint: " + rotSetpoint);

    if ((Math.abs(SwerveSubsystem.getInstance().getPose().getX() - xSetpoint) < m_tolerance) && 
        (Math.abs(SwerveSubsystem.getInstance().getPose().getY() - ySetpoint) < m_tolerance) && 
        (Math.abs(SwerveSubsystem.getInstance().getHeading().getRadians() - rotSetpoint) < Math.PI / 8.0)) {
          
      SwerveSubsystem.getInstance().stopMotors();
      if (timerStarted == false) {
        timerStarted = true;
        m_timer.restart();
      }
      
    } else {
      m_timer.stop();
      m_timer.reset();
      timerStarted = false;
      SwerveSubsystem.getInstance().drive(
        xSpeed, 
        ySpeed, 
        rot, 
        true, 
        false
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (pidX.atSetpoint() && pidY.atSetpoint() && pidRot.atSetpoint()) {
    // if (m_finishedWhenAligned && (Math.abs(SwerveSubsystem.getInstance().getPose().getX() - xSetpoint) < m_tolerance) && 
    //     (Math.abs(SwerveSubsystem.getInstance().getPose().getY() - ySetpoint) < m_tolerance) && 
    //     (Math.abs(SwerveSubsystem.getInstance().getHeading().getRadians() - rotSetpoint) < Math.PI / 8.0)) {
    //   return(true);
    // }

    if (m_finishedWhenAligned && m_timer.hasElapsed(0.4)) {
      return true;
    }

    return false;
  }
}
