// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ResetOdometry;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoSelector {
    private SendableChooser<AutoRoutine> m_autonomousRoutines;

    public enum AutoRoutine {
        DoNothing(-1),
        UseSelectorSwitch(0),
        Test1(1),
        Test2(2),
        Test3(3),
        Test4(4),
        Test5(5),

        RapidReactTest(8),
        MeasureableTest(9),

        UseFRCDashBoard(100);

        private int id;

        private AutoRoutine(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }
    }

    public AutoSelector() {
        prepareSelector();
        AutoTrajectories.constructTrajectories();
    }

    private void prepareSelector() {
        // Create Auto Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        m_autonomousRoutines = new SendableChooser<AutoRoutine>();

        // Add autonomous commands to page
        m_autonomousRoutines.setDefaultOption("Use Selector Switch", AutoRoutine.UseSelectorSwitch);
        m_autonomousRoutines.addOption("DO NOTHING", AutoRoutine.DoNothing);
        m_autonomousRoutines.addOption(AutoRoutine.Test1.toString(), AutoRoutine.Test1);
        m_autonomousRoutines.addOption(AutoRoutine.Test2.toString(), AutoRoutine.Test2);
        m_autonomousRoutines.addOption(AutoRoutine.Test3.toString(), AutoRoutine.Test3);
        m_autonomousRoutines.addOption(AutoRoutine.Test4.toString(), AutoRoutine.Test4);
        m_autonomousRoutines.addOption(AutoRoutine.Test5.toString(), AutoRoutine.Test5);

        m_autonomousRoutines.addOption(AutoRoutine.RapidReactTest.toString(), AutoRoutine.RapidReactTest);
        m_autonomousRoutines.addOption(AutoRoutine.MeasureableTest.toString(), AutoRoutine.MeasureableTest);

        tab.add("Routines", m_autonomousRoutines).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
    }


    public Command getAutoCommand() {

        AutoRoutine selectedRoutineID = getAutoRoutine();
        Trajectory[] trajs;
        
        switch (selectedRoutineID) {

            case DoNothing:
                return new InstantCommand();

            // This case should not show up here
            case UseSelectorSwitch:
                return new InstantCommand();
                
            // 
            case Test1:
                trajs = AutoTrajectories.getTrajectories(AutoRoutine.Test1);
                if (trajs == null) 
                    return new InstantCommand();

                
                return new SequentialCommandGroup(
                    new ResetOdometry(trajs[0].getInitialPose()),
                    new SwerveCommandMerge(trajs[0], new Rotation2d(Math.PI/2)),
                    new InstantCommand(SwerveSubsystem.getInstance()::stopMotors)
                );

            //
            case Test2:
                trajs = AutoTrajectories.getTrajectories(AutoRoutine.Test2);
                if (trajs == null) 
                    return new InstantCommand();

                return new SequentialCommandGroup(
                    new ResetOdometry(trajs[0].getInitialPose()),
                    new SwerveCommandMerge(trajs[0], trajs[0].getInitialPose().getRotation()),
                    new InstantCommand(SwerveSubsystem.getInstance()::stopMotors)
                );

            //
            case Test3:
                trajs = AutoTrajectories.getTrajectories(AutoRoutine.Test3);
                if (trajs == null) 
                    return new InstantCommand();

                return new SequentialCommandGroup(
                    new ResetOdometry(trajs[0].getInitialPose()),
                    new SwerveCommandMerge(trajs[0], trajs[0].getInitialPose().getRotation()),
                    new InstantCommand(SwerveSubsystem.getInstance()::stopMotors)
                );

            //
            case Test4:
                trajs = AutoTrajectories.getTrajectories(AutoRoutine.Test4);
                if (trajs == null) 
                    return new InstantCommand();

                return new SequentialCommandGroup(
                    new ResetOdometry(trajs[0].getInitialPose()),
                    new SwerveCommandMerge(trajs[0], trajs[0].getInitialPose().getRotation()),
                    new InstantCommand(SwerveSubsystem.getInstance()::stopMotors)
                );

            //
            case Test5:
                trajs = AutoTrajectories.getTrajectories(AutoRoutine.Test5);
                if (trajs == null) 
                    return new InstantCommand();

                return new SequentialCommandGroup(
                    new ResetOdometry(trajs[0].getInitialPose()),
                    new SwerveCommandMerge(trajs[0], trajs[0].getInitialPose().getRotation()),
                    new InstantCommand(SwerveSubsystem.getInstance()::stopMotors)
                );

            //
            case RapidReactTest:
                trajs = AutoTrajectories.getTrajectories(AutoRoutine.RapidReactTest);
                if (trajs == null) 
                    return new InstantCommand();
                
                Rotation2d fixedHeading = trajs[0].getInitialPose().getRotation();
                return new SequentialCommandGroup(
                    new ResetOdometry(trajs[0].getInitialPose()),
                    new SwerveCommandMerge(trajs[0], fixedHeading),
                    new WaitCommand(2).raceWith(new RunCommand(SwerveSubsystem.getInstance()::stopMotors)),
                    new SwerveCommandMerge(trajs[1], fixedHeading),
                    new WaitCommand(2).raceWith(new RunCommand(SwerveSubsystem.getInstance()::stopMotors)),
                    new SwerveCommandMerge(trajs[2], fixedHeading),
                    new InstantCommand(SwerveSubsystem.getInstance()::stopMotors)
                );
              
            //
            default:
                return new InstantCommand();
        }
        
        
    }

    /**
     * Get the desired Auto Rountine ID
     * 
     * Shuffleboard overrides all options unless it has "Use Selector Switch" active.
     * 
     * @return The ID of the desired Auto Routine
     */
    private AutoRoutine getAutoRoutine() {
        System.out.println(m_autonomousRoutines.getSelected().toString());
        if (m_autonomousRoutines.getSelected() == AutoRoutine.UseSelectorSwitch) {
            return getPathIDFromSwitch();
        }
        else if (m_autonomousRoutines.getSelected() == AutoRoutine.UseFRCDashBoard)
        {
            return getPathIDFromFRCDashboard();
        }

        return m_autonomousRoutines.getSelected();
    }

    /**
     * TODO: Implement this function
     * @return The ID of the desired Auto Routine
     */
    private AutoRoutine getPathIDFromSwitch() {
        return AutoRoutine.DoNothing;
    }

    /**
     * Use FRC Labview Dashboard
     * @return The ID of the selected Auto Routine
     */
    private AutoRoutine getPathIDFromFRCDashboard() {
        AutoRoutine autoId = AutoRoutine.DoNothing;
        String autoName = SmartDashboard.getString("Auto Selector", "Test!");
        switch(autoName)
        {
            case "Test1":
                autoId = AutoRoutine.Test1;
                break;
            case "Test2":
                autoId = AutoRoutine.Test2; 
                break;
            case "Test3":
                autoId = AutoRoutine.Test3;
                break;
            default:      
                autoId = AutoRoutine.DoNothing;
                break;
        }

        return autoId;
    }
}
