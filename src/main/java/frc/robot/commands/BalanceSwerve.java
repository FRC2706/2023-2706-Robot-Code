// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceSwerve extends CommandBase {

  // Servo servoCam = new Servo(0);

  double balanceSpeedSlow = 0.24;
  double balanceSpeed = 0.35;
  PIDController pid = new PIDController(2, 0, 0);
  double initialAngle = 0;

  /** Creates a new Balance. */
  public BalanceSwerve() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Find the reset method
    initialAngle = SwerveSubsystem.getInstance().getPitch().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveSubsystem.getInstance().drive(
      pid.calculate(SwerveSubsystem.getInstance().getPitch().getRadians(), 0),  //could be x or y
      0,
      0, 
      true, 
      true
    );


/*
 * 
 * This is the other Balance code, which doesn't use PID
 * 
 * 
 */
//     if (SwerveSubsystem.getInstance().getPitchValue() >= initialAngle - 0.05 && SwerveSubsystem.getInstance().getPitchValue() <= initialAngle + 0.05) {
//       SwerveSubsystem.getInstance().drive(0, 0, 0, false, true);
//     }
//     else if (SwerveSubsystem.getInstance().getPitchValue() <= initialAngle + 3) {
//       SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow - (SwerveSubsystem.getInstance().getPitch()) / 15, 0, false, true);
//     }
//     else if (SwerveSubsystem.getInstance().getPitchValue() < initialAngle + 10) {
//         if (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
//             SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
//         }
//         else if (SwerveSubsystem.getInstance().getPitchValue() < initialAngle) {
//             SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
//         }
//     }
//      else {
//         if (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
//             while (SwerveSubsystem.getInstance().getPitchValue() > initialAngle + 10) {
//                 SwerveSubsystem.getInstance().drive(0, balanceSpeed, 0, false, true);
//             }
//             while (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
//                 SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
//             }
//             while (SwerveSubsystem.getInstance().getPitchValue() < initialAngle) {
//                 SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
//             }
//         }
//         else {
//             while (SwerveSubsystem.getInstance().getPitchValue() < initialAngle - 10) {
//               SwerveSubsystem.getInstance().drive(0, -balanceSpeed, 0, false, true);
//             }
//             while (SwerveSubsystem.getInstance().getPitchValue() < initialAngle) {
//               SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
//             }
//             while (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
//               SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
//             }
//         }
//       }
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