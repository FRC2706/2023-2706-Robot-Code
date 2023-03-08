// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CheckArmSetpoints is designed to run in the simulation tool to display all the setpoints
 * It can be used to make sure the inital X, Z values picked are not terrible wrong.
 * This should NOT run in a real match.
 */
public class CheckArmSetpoints extends CommandBase {
    private final int TIME_PER_SETPOINT = 2;
    Timer timer = new Timer();
    ArmSetpoint[] setpoints;
    int index;

    /** Creates a new CheckArmSetpoints. */
    public CheckArmSetpoints() {
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
        timer.restart();        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.hasElapsed(TIME_PER_SETPOINT) && index < setpoints.length) {
            timer.reset();

            ArmSetpoint setpoint = setpoints[index];

            System.out.println("Displaying " + setpoint.name());

            double[] angles = ArmSubsystem.getInstance().inverseKinematics(ArmConfig.L1, ArmConfig.L2, setpoint.getX(), setpoint.getZ());

            ArmSubsystem.getInstance().updateSetpointDisplay(angles[0], angles[1]);

            index++;
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
