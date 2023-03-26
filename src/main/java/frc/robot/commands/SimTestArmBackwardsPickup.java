// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmWaypoint;

public class SimTestArmBackwardsPickup extends CommandBase {

    Timer m_timer = new Timer();
    int index = 0;
    double[][] angles;
    ArmSetpoint setpoint;

    /** Creates a new SimTestArmBackwardsPickup. */
    public SimTestArmBackwardsPickup() {
        setpoint = ArmSetpoint.ULTAUTO_BACKWARDS_PICKUP;
        angles = new double[2][setpoint.getWaypoint().length+1];

        for (int i = 0; i < setpoint.getWaypoint().length; i++) {
            ArmWaypoint waypoint = setpoint.getWaypoint()[i];
            if (waypoint.isAnglesNotXY()) {
                angles[0][i] = waypoint.getX();
                angles[1][i] = waypoint.getZ();
            } else {
                double[] new_angles = ArmSubsystem.getInstance().inverseKinematics(
                    ArmConfig.L1, ArmConfig.L2, waypoint.getX(), waypoint.getZ());

                angles[0][i] = new_angles[0];
                angles[1][i] = new_angles[1];
            }
        }

        // Main Setpoint:
        double[] new_angles = ArmSubsystem.getInstance().inverseKinematics(
                    ArmConfig.L1, ArmConfig.L2, -1 * setpoint.getX(), setpoint.getZ());

        angles[0][setpoint.getWaypoint().length] = Math.toRadians(180) - new_angles[0];
        angles[1][setpoint.getWaypoint().length] = (Math.toRadians(180) - new_angles[1]) + Math.toRadians(180);

        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();
        index = 0;

        for (int i = 0; i < setpoint.getWaypoint().length+1; i++) {
            System.out.printf("Index: %d, Bot: %.2f, Top: %.2f \n", i, Math.toDegrees(angles[0][i]), Math.toDegrees(angles[1][i]));
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_timer.hasElapsed(3)) {
            m_timer.reset();
            System.out.printf("\n~~ Index: %d\n", index);

            if (index < ArmSetpoint.ULTAUTO_BACKWARDS_PICKUP.getWaypoint().length+1) {
                ArmSubsystem.getInstance().updateSetpointDisplay(
                    angles[0][index], 
                    angles[1][index]
                );
                index++;
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
        return index >= ArmSetpoint.ULTAUTO_BACKWARDS_PICKUP.getWaypoint().length+1;
    }
}
