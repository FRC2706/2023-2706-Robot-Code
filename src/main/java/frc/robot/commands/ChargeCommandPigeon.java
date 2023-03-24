// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * ChargeCommand is a command that has 3 states which it cycles through one by one until it reaches the final state.
 * 
 * State0:
 *  - It starts in state 0. 
 *  - The condition to move to state 1 is when the gyro pitch/roll value goes above a certain value. 
 *  - This condition means the robot has changed from driving along the floor to driving up the ramp of the charge station.
 * 
 * State1:
 *  - It changes from state 1 to state 2 when the gyro pitch/roll value goes back below a certain value. 
 *  - This means the robot has gone past the tipping point of the charging station and the charge station is almost balanced.
 * 
 * State2:
 *  - State 2 only runs for a short period of time (~0.1 on March23) and it drives backwards slightly to get rid of all the robot's speed.
 * 
 * After the command ends, it should be in a SequentialCommandGroup to then schedule a ChargeStationLock command in order to lock the
 *    wheels in a pattern that prevents it from rolling off the charging station if it's not balanced.
 */
public class ChargeCommandPigeon extends CommandBase {
  Timer m_timer = new Timer();

  ProfiledPIDController pidControlTheta;
  double currentTheta;
  double desiredTheta;

  double pigeonValue;
  double initPigeonValue;

  double initXPos;
  double currentX;
  double xLimit;

  boolean bWideSide;
  double flipXDirection;

  // Erik: This 10 degrees was tested to work well with the old ChargeCommand and should still be good.
  //           Only change this is it's not properly going through the states but preferably don't touch it.
  double PITCH_TOLERANCE = 10;

  // Erik: These are just safety factors to make sure the robot doesn't drive to the other side of the 
  //          field if something goes wrong. 4.5 meters is perfect.
  double X_WIDE_LIMIT = 4.5;
  double X_NARROW_LIMIT = 4.5;

  /**
   * Erik Tuning suggestions for ChargeCommandPigeon
   * 
   * Note:
   * - You want to tune this command such that it stops in the MIDDLE of the charging station.
   *      Even if it's balanced that's not good enough. It should be tuned so that it stops in
   *      the middle.
   * 
   * Tips:
   * - Special case: It's possible X_SPEED_STATE0 is too high if it never decelerates to X_SPEED_STATE1 
   *        before reaching X_SPEED_STATE2. If this is true, decrease X_SPEED_STATE0.
   * 
   * - If it fails to tip the charging station, increase X_SPEED_STATE0 (but I expect this won't be needed)
   * 
   * 
   * ~ If it's gone too far, do one of the following:
   *    - Increase the "jump back", aka increase either TIME_FOR_REVERSING or X_SPEED_STATE2.
   *    - Decrease X_SPEED_STATE1 to approach the center at a slower rate.
   * 
   * ~ If it's not gone far enough, do one of the following:
   *    - Increase X_SPEED_STATE1 to climb up the ramp at a faster speed, making it go farther before the "jump back" happens.
   *    - If you have a huge "jump back", reduce the "jump back". If you have a small "jump back" then leave it alone.
   *      - Decrease either TIME_FOR_REVERSING or X_SPEED_STATE2 to reduce "jump back".
   *    
   *   
   * There are 2 ways to approach tuning this command:
   * ~ Method 1: Slow but consistent:
   *    - Tune X_SPEED_STATE1 to slowly crawl up the ramp.
   *    - Have a tiny jump back that does almost nothing (just gets rid of most the speed)
   * 
   * ~ Method 2: Save 1 to 3 seconds but a little chaotic
   *    - Tune X_SPEED_STATE1 to drive up the ramp faster than a slow crawl
   *    - Tune X_SPEED_STATE1 to be fast enough it ends up going slightly too far
   *    - Tune the "jump back" actually jump back, getting rid of all speed from X_SPEED_STATE1 and
   *          have it go back by 0.1 meters.
   * 
   * The best way will probably be mostly method 1 with a bit of method 2.
   */

  // Unit: seconds
  double TIME_FOR_REVERSING = 0.10;

  // Unit: meters per second
  double X_SPEED_STATE0 = 1.7;
  double X_SPEED_STATE1 = 0.7;
  double X_SPEED_STATE2 = -0.5; // Negative because it should "jump back" or just get rid of any velocity.

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
      xLimit = X_WIDE_LIMIT;

    }
    else
    {
      //use the pitch value
      initPigeonValue = SwerveSubsystem.getInstance().getPitch();
      xLimit = X_NARROW_LIMIT;
    }

    state = 0;
    initXPos = SwerveSubsystem.getInstance().getPose().getX();

    flipXState0 = flipXDirection;
    flipXState1 = flipXDirection;
    flipXState2 = flipXDirection;

    m_timer.stop();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTheta = SwerveSubsystem.getInstance().getHeading().getRadians();
    double theta = pidControlTheta.calculate(currentTheta, desiredTheta);

    currentX = SwerveSubsystem.getInstance().getPose().getX();

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
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((state == 2 && m_timer.hasElapsed(TIME_FOR_REVERSING)) || Math.abs(initXPos - currentX) > xLimit)
      return (true);
    else
      return (false);
  }
}
