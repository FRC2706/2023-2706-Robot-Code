// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmDisplay;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmWaypoint;

public class ArmCommand extends CommandBase {
  ArmSetpoint armSetpoint;
  double defaultAngle1 = Math.PI; // angle in radians of joint 1 in default position (vertical)
  double defaultAngle2 = Math.PI/36; // angle in radians of joint 2 in default position (angle of 5 degrees)
  double[] defaultAngles;
  double[] angles;
  double angle1;
  double angle2;
  int index;
  double tempX;
  double tempZ;

  // arm simulation variables
  ArmDisplay armDisplay;

  boolean startBrakeTimer;
  Timer m_timer = new Timer();
  

  /** Creates a new ArmExtend. */
  
  public ArmCommand(ArmSetpoint armSetpoint) {
    this.armSetpoint = armSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("~~~~~" + armSetpoint.name());

    if (Double.isNaN(armSetpoint.getX()) && Double.isNaN(armSetpoint.getZ())) {
      DriverStation.reportError("ArmSetpoint called " + armSetpoint.name() + " is NaN", true);
      this.cancel();
    }

    if (armSetpoint.getWaypoint().length == 0) {
      index = 99;
    } else {
      index = 0;
    }
    
    ArmSubsystem.getInstance().resetMotionProfile();
    ArmSubsystem.getInstance().controlTopArmBrake(false);
    ArmSubsystem.getInstance().controlBottomArmBrake(false);

    startBrakeTimer = false;
    m_timer.stop();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmWaypoint waypoint;

    if (index >= 99) {
      tempX = armSetpoint.getX();
      tempZ = armSetpoint.getZ();
    }
    else {
      waypoint = armSetpoint.getWaypoint()[index];
      tempX = waypoint.getX();
      tempZ = waypoint.getZ();
    }

    angles = ArmSubsystem.getInstance().inverseKinematics(ArmConfig.L1, ArmConfig.L2, tempX, tempZ);
    angle1 = angles[0];
    angle2 = angles[1];


    ArmSubsystem.getInstance().updateSetpointDisplay(angle1, angle2);

    ArmSubsystem.getInstance().setTopJoint(angle2);
    ArmSubsystem.getInstance().setBottomJoint(angle1, angle2);

    boolean topReached = Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.positionTolerance &&
                              Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.velocityTolerance;
    boolean bottomReached = Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.positionTolerance &&
                                  Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.velocityTolerance;
    if (index >= 99) {
      if (topReached) {
        ArmSubsystem.getInstance().controlTopArmBrake(true);
      }
      if (bottomReached) {
        ArmSubsystem.getInstance().controlBottomArmBrake(true);
      }
      if (startBrakeTimer == false && topReached && bottomReached) {
        startBrakeTimer = true;
        m_timer.restart();
      }
      if (startBrakeTimer && armSetpoint == ArmSetpoint.PICKUP) {
        ArmSubsystem.getInstance().testFeedForwardTop(-10);
      }
      if (startBrakeTimer && armSetpoint == ArmSetpoint.TOP_CONE) {
        if (ArmSubsystem.getInstance().getTopPosition() < angle2) {
          ArmSubsystem.getInstance().testFeedForwardTop(2.0);
        }
      }
    } else {
      topReached = Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.waypointPositionTolerance &&
          Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.waypointVelocityTolerance;
      bottomReached = Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.waypointPositionTolerance &&
          Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.waypointVelocityTolerance;
      if (bottomReached && topReached) {
        if (index == armSetpoint.getWaypoint().length - 1 || armSetpoint.getWaypoint().length == 0) {
          index = 99;
          // tempX = armSetpoint.getX();
          // tempZ = armSetpoint.getZ();
          // angles = ArmSubsystem.getInstance().inverseKinematics(ArmConfig.L1, ArmConfig.L2, tempX, tempZ);
          // angle1 = angles[0];
          // angle2 = angles[1];
        }
        else {
          index += 1;
        }
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
    if (interrupted == false) {
      ArmSubsystem.getInstance().controlTopArmBrake(true);
      ArmSubsystem.getInstance().controlBottomArmBrake(true);
    }

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.positionTolerance &&
    //  Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.positionTolerance &&
    //  Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.velocityTolerance &&
    //   Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.velocityTolerance && index >= 99) {
    //     return true;
    //   }
    // else {
    //   return false;
    // }
    if (armSetpoint == ArmSetpoint.PICKUP) {
      return m_timer.hasElapsed(0.4);
    }
    return m_timer.hasElapsed(0.2);
  }
}
