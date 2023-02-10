// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.config.Config;

public class BalanceSwerve extends CommandBase {

  double balanceSpeedSlow = 0.25;
  double balanceSpeed = 0.35;
  PIDController pid = new PIDController(Config.Swerve.drive_kP, Config.Swerve.drive_kI, Config.Swerve.drive_kD);
  double initialAngle = 0;


  /** Creates a new BalanceSwerve. */
  public BalanceSwerve() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.getInstance().resetPigeon();
    initialAngle = SwerveSubsystem.getInstance().getRollValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: tune balanceSpeed
    if (initialAngle >= 0) {
      SwerveSubsystem.getInstance().drive(Math.sin((SwerveSubsystem.getInstance().getRollValue() - initialAngle)) * balanceSpeed, 0, 0, false, true);
    }
    else {
      SwerveSubsystem.getInstance().drive(Math.sin((SwerveSubsystem.getInstance().getRollValue() + initialAngle)) * balanceSpeed, 0, 0, false, true);
    }
/* 
     if (SwerveSubsystem.getInstance().getRollValue() >= initialAngle - 0.05 && SwerveSubsystem.getInstance().getRollValue() <= initialAngle + 0.05) {
       SwerveSubsystem.getInstance().drive(0, 0, 0, false, true);
     }
     else if (SwerveSubsystem.getInstance().getRollValue() <= initialAngle + 3) {
      System.out.println("Initial phase");
       SwerveSubsystem.getInstance().drive(balanceSpeedSlow - (SwerveSubsystem.getInstance().getRollValue()) / 15, 0, 0, false, true);
     }
     else if (SwerveSubsystem.getInstance().getRollValue() < initialAngle + 10) {
      System.out.println("Angled phase");
         if (SwerveSubsystem.getInstance().getRollValue() > initialAngle) {
             SwerveSubsystem.getInstance().drive(pid.calculate(balanceSpeedSlow), 0, 0, false, true);
         }
         else if (SwerveSubsystem.getInstance().getRollValue() < initialAngle) {
             SwerveSubsystem.getInstance().drive(pid.calculate(-balanceSpeedSlow), 0, 0, false, true);
         }
     }
      else {
         if (SwerveSubsystem.getInstance().getRollValue() > initialAngle) {
             while (SwerveSubsystem.getInstance().getRollValue() > initialAngle + 10) {
                 SwerveSubsystem.getInstance().drive(pid.calculate(balanceSpeed), 0, 0, false, true);
             }
             while (SwerveSubsystem.getInstance().getRollValue() > initialAngle) {
                 SwerveSubsystem.getInstance().drive(pid.calculate(balanceSpeedSlow), 0, 0, false, true);
             }
             while (SwerveSubsystem.getInstance().getRollValue() < initialAngle) {
                 SwerveSubsystem.getInstance().drive(pid.calculate(-balanceSpeedSlow), 0, 0, false, true);
             }
         }
         else {
             while (SwerveSubsystem.getInstance().getRollValue() < initialAngle - 10) {
               SwerveSubsystem.getInstance().drive(pid.calculate(-balanceSpeed), 0, 0, false, true);
             }
             while (SwerveSubsystem.getInstance().getRollValue() < initialAngle) {
               SwerveSubsystem.getInstance().drive(pid.calculate(-balanceSpeedSlow), 0, 0, false, true);
             }
             while (SwerveSubsystem.getInstance().getRollValue() > initialAngle) {
               SwerveSubsystem.getInstance().drive(pid.calculate(balanceSpeedSlow), 0, 0, false, true);
             }
         }
       }
       */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
