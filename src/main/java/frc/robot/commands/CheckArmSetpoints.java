// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmWaypoint;

/**
 * CheckArmSetpoints is designed to run in the simulation tool to display all the setpoints
 * It can be used to make sure the inital X, Z values picked are not terrible wrong.
 * This should NOT run in a real match.
 */
public class CheckArmSetpoints extends CommandBase {
    private final double TIME_PER_SETPOINT = 0.5;
    Timer timer = new Timer();
    ArmSetpoint[] setpoints;
    int index;
    int waypointIndex;
    CommandXboxController m_joystick;

    /** Creates a new CheckArmSetpoints. */
    public CheckArmSetpoints(CommandXboxController joystick) {
        m_joystick = joystick;
        
        setpoints = new ArmSetpoint[]{
            ArmSetpoint.HOME_WITH_GAMEPIECE,
            ArmSetpoint.PICKUP,
            ArmSetpoint.PICKUP_OUTSIDE_FRAME,
            ArmSetpoint.HUMAN_PLAYER_PICKUP,

            ArmSetpoint.BOTTOM_CONE,
            ArmSetpoint.MIDDLE_CONE,
            ArmSetpoint.MIDDLE_CONE_RELEASE,
            ArmSetpoint.TOP_CONE,
            ArmSetpoint.TOP_CONE_RELEASE,

            ArmSetpoint.BOTTOM_CUBE,
            ArmSetpoint.MIDDLE_CUBE,
            ArmSetpoint.TOP_CUBE,
        };
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        index = 0;
        waypointIndex = 0;
        timer.restart();        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (timer.hasElapsed(TIME_PER_SETPOINT) && index < setpoints.length) {
        if (timer.hasElapsed(TIME_PER_SETPOINT) && index < setpoints.length && m_joystick.x().getAsBoolean()) {
            timer.reset();
            
            ArmSetpoint setpoint = setpoints[index];

            double x, z;
            if (waypointIndex == 99 || setpoint.getWaypoint().length == 0) {
                x = setpoint.getX();
                z = setpoint.getZ();
                waypointIndex = 99;
            } else {
                ArmWaypoint waypoint = setpoint.getWaypoint()[waypointIndex];
                x = waypoint.getX();
                z = waypoint.getZ();
                waypointIndex++;
            }
            
            

            double[] angles = ArmSubsystem.getInstance().inverseKinematics(ArmConfig.L1, ArmConfig.L2, x, z);

            ArmSubsystem.getInstance().updateSetpointDisplay(angles[0], angles[1]);

            if (waypointIndex-1 == 0) {
                System.out.println("\n\nDisplaying " + setpoint.name());
            }
            if (waypointIndex != 99) {
                System.out.printf("Waypoint: %d, bottomAngle: %.2f, topAngle: %.2f \n", waypointIndex-1, Math.toDegrees(angles[0]), Math.toDegrees(angles[1]));
            } else {
                System.out.printf("Setpoint: %s, bottomAngle: %.2f, topAngle: %.2f \n", setpoint.name(), Math.toDegrees(angles[0]), Math.toDegrees(angles[1]));
            }

            if (waypointIndex == 99) {
                index++;
                waypointIndex = 0;
            } else if (waypointIndex == setpoint.getWaypoint().length) {
                waypointIndex = 99;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return index >= setpoints.length;
    }
}
