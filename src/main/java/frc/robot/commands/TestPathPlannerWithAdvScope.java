// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class TestPathPlannerWithAdvScope extends CommandBase {
    private BaseAutoBuilder m_autoBuilder;
    private IntegerEntry entryGetPathIndex;
    private DoubleEntry entryVel, entryAccel;
    private Command pathplannerCommand = new InstantCommand();
    
    private final String[] PATH_NAMES_TO_TEST = {
        "KingstonRedNonBump",
        "KingstonRedBumpJustDriving",
        "KingstonRedBump",
        "KingstonRedLooping"
    };

    /** Creates a new TestPathPlannerWithAdvScope. */
    public TestPathPlannerWithAdvScope(BaseAutoBuilder autoBuilder) {
        m_autoBuilder = autoBuilder;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("TestPathPlanner");
        entryGetPathIndex = table.getIntegerTopic("PathIndexToTest").getEntry(2);
        entryGetPathIndex.set(-1);

        entryVel = table.getDoubleTopic("Vel").getEntry(2.5);
        entryVel.set(1);

        entryAccel = table.getDoubleTopic("Accel").getEntry(3);
        entryAccel.set(1);

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        int index = (int) entryGetPathIndex.get();
        if (index < 0 || index >= PATH_NAMES_TO_TEST.length) {
            System.out.println("PathIndexToTest is " + index + " which is not valid for TestPathPlannerWithAdvScope");
            this.cancel();
        } else {
            pathplannerCommand = m_autoBuilder.fullAuto(
                PathPlanner.loadPathGroup(
                    PATH_NAMES_TO_TEST[index], 
                    entryVel.get(),   // 2.5
                    entryAccel.get()   // 3
                )
            );
    
            pathplannerCommand.initialize();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pathplannerCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pathplannerCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pathplannerCommand.isFinished();
    }
}
