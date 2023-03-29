// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmDisplay;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmWaypoint;

public class ArmJoystickConeCommand extends CommandBase {
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
  Timer m_timer2 = new Timer();

  // joystick value controlling cone arm
  private CommandXboxController m_joystick;
  double z_offset = 0;
  Debouncer m_topDebounce = new Debouncer(0.3);
  Debouncer m_bottomDebounce = new Debouncer(0.3);

  boolean topBrakeOn = false;
  boolean botBrakeOn = false;
  Timer topBrakeTimer = new Timer();
  Timer botBrakeTimer = new Timer();
  

  /** Creates a new ArmExtend. */
  
  public ArmJoystickConeCommand(ArmSetpoint armSetpoint, CommandXboxController operator_stick) {
    this.armSetpoint = armSetpoint;
    this.m_joystick = operator_stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("~~~~~" + armSetpoint.name());

    if (Double.isNaN(armSetpoint.getX()) || Double.isNaN(armSetpoint.getZ())) {
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

    if (armSetpoint == ArmSetpoint.MIDDLE_CONE) {
      ArmSubsystem.getInstance().setTopConstraints(ArmConfig.TOP_CONE_MIDDLE_MAX_VEL, ArmConfig.TOP_CONE_MIDDLE_MAX_ACCEL);
    }
    if (armSetpoint == ArmSetpoint.TOP_CONE) {
      ArmSubsystem.getInstance().setTopConstraints(ArmConfig.TOP_CONE_TOP_MAX_VEL, ArmConfig.TOP_CONE_TOP_MAX_ACCEL);
    }

    startBrakeTimer = false;
    m_timer.stop();
    m_timer.reset();
    m_timer2.stop();
    m_timer2.reset();

    topBrakeTimer.stop();
    topBrakeTimer.reset();
    botBrakeTimer.stop();
    botBrakeTimer.reset();

    z_offset = 0;

    m_topDebounce.calculate(false);
    m_bottomDebounce.calculate(false);

    topBrakeOn = false;
    botBrakeOn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmWaypoint waypoint;

    if (index >= 99) {
      tempX = armSetpoint.getX() + z_offset * -0.3;
      tempZ = armSetpoint.getZ() + z_offset;

      double z = m_joystick.getRawAxis(XboxController.Axis.kRightY.value);
      z = MathUtil.applyDeadband(z, ArmConfig.ARM_JOYSTICK_DEADBAND);
      z_offset += z * -0.16;
      z_offset = MathUtil.clamp(z_offset, -8, 0); // Currently allowing the right joystick to control the arm up to 8 inches downwards. Do not change the 0.

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

    if (!(topBrakeOn && topBrakeTimer.hasElapsed(0.3))) {
      ArmSubsystem.getInstance().setTopJoint(angle2);
    } else {
      ArmSubsystem.getInstance().m_topArm.stopMotor();
    }

    if (!(botBrakeOn && botBrakeTimer.hasElapsed(0.3))) {
      ArmSubsystem.getInstance().setBottomJoint(angle1, angle2);
    } else {
      ArmSubsystem.getInstance().m_bottomArm.stopMotor();
    }
   

    boolean topReached = Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.positionTolerance &&
                              Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.velocityTolerance;
    boolean bottomReached = Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.positionTolerance &&
                                  Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.velocityTolerance;

   

    topReached = m_topDebounce.calculate(topReached);
    bottomReached = m_bottomDebounce.calculate(bottomReached);

    if (index >= 99) {
      if (topBrakeOn) {
        boolean topReleaseBrake = Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.positionTolerance+Math.toRadians(1) &&
                              Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.velocityTolerance+Math.toRadians(1);
        if (topReleaseBrake == false) {
          topBrakeOn = false;
          ArmSubsystem.getInstance().controlTopArmBrake(false);
        }
      }
      else {
        if (topReached) {
          if (topBrakeOn == false) {
            topBrakeTimer.restart();
          }
          topBrakeOn = true;
          ArmSubsystem.getInstance().controlTopArmBrake(true);
        }
      }
      if (botBrakeOn) {
        boolean bottomReleaseBrake = Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.positionTolerance+Math.toRadians(1) &&
                              Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.velocityTolerance+Math.toRadians(1);
        if (bottomReleaseBrake == false) {
          botBrakeOn = false;
          ArmSubsystem.getInstance().controlBottomArmBrake(false);
        }                 
      }
      else {
        if (bottomReached) {
          if (botBrakeOn == false) {
            botBrakeTimer.restart();
          }
          botBrakeOn = true;
          ArmSubsystem.getInstance().controlBottomArmBrake(true);
        }
      }
      if (startBrakeTimer == false && topReached && bottomReached) {
        startBrakeTimer = true;
        m_timer.restart();
      }
      // if (startBrakeTimer && armSetpoint == ArmSetpoint.TOP_CONE) {
      //   if (ArmSubsystem.getInstance().getTopPosition() < angle2) {
      //     ArmSubsystem.getInstance().testFeedForwardTop(2.0);
      //   }
      // }
    } else {

      if (armSetpoint == ArmSetpoint.PICKUP && index == 1) {
        topReached = Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.waypointPickupPositionTolerance &&
          Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.waypointPickupVelocityTolerance;
        bottomReached = Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.waypointPickupPositionTolerance &&
          Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.waypointPickupVelocityTolerance;

      }
      else {
        topReached = Math.abs(ArmSubsystem.getInstance().getTopPosition() - angle2) < ArmConfig.waypointConePositionTolerance; //&&
        // Math.abs(ArmSubsystem.getInstance().getTopVel()) < ArmConfig.waypointVelocityTolerance;
        bottomReached = Math.abs(ArmSubsystem.getInstance().getBottomPosition() - angle1) < ArmConfig.waypointConePositionTolerance; //&&
        // Math.abs(ArmSubsystem.getInstance().getBottomVel()) < ArmConfig.waypointVelocityTolerance;
      }
      if (bottomReached && topReached) {
        if (index == armSetpoint.getWaypoint().length - 1 || armSetpoint.getWaypoint().length == 0) {
          index = 99;
          if (armSetpoint == ArmSetpoint.PICKUP) {
            m_timer2.restart();
          }
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
    return false;
  }
}
