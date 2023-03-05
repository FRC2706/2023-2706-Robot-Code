// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This should only be a superclass to other RobotContainers.
 * 
 * DO NOT PUT CODE IN THIS CLASS.
 * 
 */
public abstract class RobotContainer {
    public Command getAutonomousCommand() {
        return new InstantCommand();
    } 
    
}

/**
 *  **The idea of having multiple RobotContainers**
 * 
 * TDLR:
 * The idea is to have multiple robot containers to control the commands, button bindings and auto command for each robot individually.
 * This prevents unwanted subsystems and unwanted hardware objects (like TalonSRX or SparkMax) from being constructed.
 * 
 * 
 *  Idea:
 *  We want to write code that works on multiple robots.
 * 
 *  Problem: 
 *      Some devices like the TalonSRX or SparkMax can cause problems if they are
 *          constructed by never find the canID for that device on the canBus.
 * 
 *  Solution:
 *      To prevent hardware objects from being constructed, we prevent their Subsystems
 *          from being construction.
 * 
 *      To prevent the subsystems from being constructed, we prevent the related commands
 *          from being construction.
 * 
 *      To control what commands are constructed, we have multiple RobotContainers that can
 *          define their own set of Commands, ButtonBindings, Autononaous command. 
 * 
 *      
 *  SubsystemChecker.java
 *      This is a tool to make sure this idea goes smootly.
 * 
 *      SubsystemChecker.java must be told when a Subsystem is constructed.
 * 
 *      Use the SubsystemChecker.subsystemConstructed function to tell the checker.
 *          This function must be told the type of subsystem too.
 *          Use SubsystemChecker.SubsystemType for the type
 * 
 *      Add types to SubsystemType if you are making a new Subsystem.
 *  
 *      Add the allowed SubsystemType for each robot ID in their related arrays.
 *
 */