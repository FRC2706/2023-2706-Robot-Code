// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetBlingCommand;
import frc.robot.commands.TranslationCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */

public class AutoRoutines {
    SwerveAutoBuilder autoBuilder;

    List<PathPlannerTrajectory> Practice1;
    List<PathPlannerTrajectory> Practice2;
    List<PathPlannerTrajectory> leave_top;
    List<PathPlannerTrajectory> leave_middle_around_;
    List<PathPlannerTrajectory> leave_middle_through_;
    List<PathPlannerTrajectory> leave_bottom;
    List<PathPlannerTrajectory> leave_balance_top;    
    List<PathPlannerTrajectory> leave_balance_middle_around_;
    List<PathPlannerTrajectory> leave_balance_middle_through_;
    List<PathPlannerTrajectory> leave_balance_bottom;
    List<PathPlannerTrajectory> place_pick_top;
    List<PathPlannerTrajectory> place_pick_middle_around_;
    List<PathPlannerTrajectory> place_pick_middle_through_;
    List<PathPlannerTrajectory> place_pick_bottom;
    List<PathPlannerTrajectory> place_pick_balance_top;
    List<PathPlannerTrajectory> place_pick_balance_middle;
    List<PathPlannerTrajectory> place_pick_balance_bottom;
    List<PathPlannerTrajectory> place_pick_place_balance_top;
    List<PathPlannerTrajectory> place_pick_place_balance_middle;
    List<PathPlannerTrajectory> place_pick_place_balance_bottom;
    List<PathPlannerTrajectory> place_pick_place_top;
    List<PathPlannerTrajectory> place_pick_place_middle;
    List<PathPlannerTrajectory> place_pick_place_bottom;
    List<PathPlannerTrajectory> place_pick_place_pick_place_top;
    List<PathPlannerTrajectory> place_pick_place_pick_place_middle;
    List<PathPlannerTrajectory> place_pick_place_pick_place_bottom;
    List<PathPlannerTrajectory> place_pick_place_pick_place_bottom_new;



    public AutoRoutines() {
        Map<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("intake", new InstantCommand());
        eventMap.put("Bling Purple", new SetBlingCommand(1));
        eventMap.put("Bling Blue", new SetBlingCommand(2));
        eventMap.put("Bling Red", new SetBlingCommand(3));
        eventMap.put("Bling Honeydew", new SetBlingCommand(4));

        autoBuilder = new SwerveAutoBuilder(
                SwerveSubsystem.getInstance()::getPose,
                SwerveSubsystem.getInstance()::resetOdometry,
                Config.Swerve.kSwerveDriveKinematics,
                new PIDConstants(5, 0, 0),
                new PIDConstants(3, 0, 0.1),
                SwerveSubsystem.getInstance()::setModuleStatesAuto,
                eventMap,
                true,
                SwerveSubsystem.getInstance());

        Practice1 = PathPlanner.loadPathGroup("Practice1", 2.5, 3);
        Practice2 = PathPlanner.loadPathGroup("Practice2", 2.5, 3);
        leave_top = PathPlanner.loadPathGroup("leave_top", 2.5, 3);
        leave_middle_around_ = PathPlanner.loadPathGroup("leave_middle_around_", 2.5, 3);
        leave_middle_through_ = PathPlanner.loadPathGroup("leave_middle_through_", 2.5, 3);
        leave_bottom = PathPlanner.loadPathGroup("leave_bottom", 2.5, 3);
        leave_balance_top = PathPlanner.loadPathGroup("leave_balance_top", 2.5, 3);
        leave_balance_middle_around_ = PathPlanner.loadPathGroup("leave_balance_middle_around_", 2.5, 3);
        leave_balance_middle_through_ = PathPlanner.loadPathGroup("leave_balance_middle_through_", 2.5, 3);
        leave_balance_bottom = PathPlanner.loadPathGroup("leave_balance_bottom", 2.5, 3);
        place_pick_top = PathPlanner.loadPathGroup("place_pick_top", 2.5, 3);
        place_pick_middle_around_ = PathPlanner.loadPathGroup("place_pick_middle_around_", 2.5, 3);
        place_pick_middle_through_ = PathPlanner.loadPathGroup("place_pick_middle_through_", 2.5, 3);
        place_pick_bottom = PathPlanner.loadPathGroup("place_pick_bottom", 2.5, 3);
        place_pick_balance_top = PathPlanner.loadPathGroup("place_pick_balance_top", 2.5, 3);
        place_pick_balance_middle = PathPlanner.loadPathGroup("place_pick_balance_middle", 2.5, 3);
        place_pick_balance_bottom = PathPlanner.loadPathGroup("place_pick_balance_bottom", 2.5, 3);
        place_pick_place_top = PathPlanner.loadPathGroup("place_pick_place_top", 2.5, 3);
        place_pick_place_balance_middle = PathPlanner.loadPathGroup("place_pick_place_balance_middle", 2.5, 3);
        place_pick_place_balance_bottom = PathPlanner.loadPathGroup("place_pick_place_balance_bottom", 2.5, 3);
        place_pick_place_top = PathPlanner.loadPathGroup("place_pick_place_top", 2.5, 3);
        place_pick_place_middle = PathPlanner.loadPathGroup("place_pick_place_middle", 2.5, 3);
        place_pick_place_bottom = PathPlanner.loadPathGroup("place_pick_place_bottom", 2.5, 3);
        place_pick_place_pick_place_top = PathPlanner.loadPathGroup("place_pick_place_pick_place_top", 2.5, 3);
        place_pick_place_pick_place_middle = PathPlanner.loadPathGroup("place_pick_place_pick_place_middle", 2.5, 3);
        place_pick_place_pick_place_bottom = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom", 2.5, 3);
        place_pick_place_pick_place_bottom_new = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom2", 3, 4);// 2.5, 3);

    }

    public Command getAutonomousCommand(int selectAuto) {
        System.out.println (selectAuto);
        switch (selectAuto) {
            case 0:
                return null;

            case 1:
                return (autoBuilder.fullAuto(leave_top));

            case 2:
                return (autoBuilder.fullAuto(leave_middle_around_));

            case 3:
                return (autoBuilder.fullAuto(leave_middle_through_));

            case 4:
                return (autoBuilder.fullAuto(leave_bottom));
            
            case 5:
                return (autoBuilder.fullAuto(leave_balance_top));

            case 6:
                return (autoBuilder.fullAuto(leave_balance_middle_around_));

            case 7:
                return (autoBuilder.fullAuto(leave_balance_middle_through_));

            case 8:
                return (autoBuilder.fullAuto(leave_balance_bottom));
                
            case 9:
                return (autoBuilder.fullAuto(place_pick_top));

            case 10:
                return (autoBuilder.fullAuto(place_pick_middle_around_));
                
            case 11:
                return (autoBuilder.fullAuto(place_pick_middle_through_));
                

            case 12:
                return (autoBuilder.fullAuto(place_pick_bottom));

            case 13:
                return (autoBuilder.fullAuto(place_pick_balance_top));

            case 14:
                return (autoBuilder.fullAuto(place_pick_balance_middle));

                //test this case
            case 15:
                return new SequentialCommandGroup(
                    autoBuilder.fullAuto(place_pick_balance_bottom),
                    new TranslationCommand(-2.7,0)
                );
            case 16:
                return (autoBuilder.fullAuto(place_pick_place_balance_top));

            case 17:
                return (autoBuilder.fullAuto(place_pick_place_balance_middle));
            
            case 18:
                return (autoBuilder.fullAuto(place_pick_place_balance_bottom));

            case 19:
                return (autoBuilder.fullAuto(place_pick_place_top));

            case 20:
                return (autoBuilder.fullAuto(place_pick_place_middle));

            case 21:
                return (autoBuilder.fullAuto(place_pick_place_bottom));

            case 22:
                return (autoBuilder.fullAuto(place_pick_place_pick_place_top));

            case 23:
                return (autoBuilder.fullAuto(place_pick_place_pick_place_middle));

            case 24:
                return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom));
            
            case 25:
                return (autoBuilder.fullAuto(Practice1));
                
            case 26:
                return (autoBuilder.fullAuto(Practice2));


            case 27:
                return autoBuilder.fullAuto(place_pick_place_pick_place_bottom_new);

        }
        return new InstantCommand();
    }
}
