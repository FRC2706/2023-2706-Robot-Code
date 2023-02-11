// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.Config;

/** Add your docs here. */
public class SubsystemChecker {

    /**
     * Name of possible subsystems
     */
    public enum SubsystemType {
        DiffNeoSubsystem,
        DiffTalonSubsystem,
        SwerveSubsystem,
        ClimberSubsystem,
        ShooterSubsystem, 
        RollerIntakeSubsystem,
        PneumaticIntakeSubsystem,
        PneumaticShooterSubsystem,
        IndexerSubsystem,
        ArmSubsystem,
        RelaySubsystem,
    };

    /**
     * Allowed Subsystems for each robot
     */
    // RobotID: 0, 2023 Competition robot, unnamed
    private static SubsystemType[] compBotId0 = new SubsystemType[] {
        SubsystemType.SwerveSubsystem,  // Chassis unknown
    };

    // RobotID: 1, 2022 robot, RapidReact, Clutch
    private static SubsystemType[] clutchId1 = new SubsystemType[] {
        SubsystemType.DiffTalonSubsystem, // Chassis

        SubsystemType.RollerIntakeSubsystem,
        SubsystemType.PneumaticIntakeSubsystem,
        SubsystemType.PneumaticShooterSubsystem,
        SubsystemType.IndexerSubsystem,
        SubsystemType.ShooterSubsystem,
        SubsystemType.ClimberSubsystem,
    };

    // RobotID: 2, Half-scale talon differential drive robot, Beetle
    private static SubsystemType[] beetleId2 = new SubsystemType[] {
        SubsystemType.SwerveSubsystem,  // Chassis
    };

    // RobotID: 3, 2019 Comp Robot, Deep Space, Mergonaut
    private static SubsystemType[] mergonautId3 = new SubsystemType[] {
        SubsystemType.DiffTalonSubsystem,  // Chassis
    };

    // RobotID: 4, 2022 Fall Half-scale swerve robot, unnamed
    private static SubsystemType[] miniSwerveId4 = new SubsystemType[] {
        SubsystemType.SwerveSubsystem, // Chassis
        SubsystemType.RelaySubsystem,
    };

    // RobotID: 5, 2022 Fall Half-scale Neo Differential drive robot, unnamed
    private static SubsystemType[] neoMiniRobotId5 = new SubsystemType[] {
        SubsystemType.DiffNeoSubsystem, // Chassis
    };

    // Use robotSpecific to know what robot is currently running the code
    private static SubsystemType[] activeRobotAllowedTypes = Config.robotSpecific(compBotId0, clutchId1, beetleId2, mergonautId3, miniSwerveId4, neoMiniRobotId5);

    /**
     * Check if the subsystem is allowed for the robot this is deployed onto
     * 
     * @param subsystem A SubsystemType to identify the subsystem that has been constructed
     */
    public static void subsystemConstructed(SubsystemType subsystem) {
         if (!canSubsystemConstruct(subsystem)) {
            throw new RuntimeException(String.format(
                "SUBSYSTEM INITALIZED - NOT ALLOWED ON THIS ROBOT - RobotID: %d, IllegalSubsystem: %s",
                Config.getRobotId(), subsystem.toString())
            );
        }
    }

    /**
     * Search the array of allowed subsystems to see if this subsystem is in it. 
     * 
     * @param allowed List of allowed Subsystems
     * @param subsystem The subsystem that is being checked
     * @return True is the subsystem is in the array
     */
    public static boolean canSubsystemConstruct(SubsystemType subsystem) {
        for (int i = 0; i < activeRobotAllowedTypes.length; i++) 
            if (activeRobotAllowedTypes[i].equals(subsystem)) 
                return true;      

        // Never found the subsystem in the array, return false
        return false;
    }
}