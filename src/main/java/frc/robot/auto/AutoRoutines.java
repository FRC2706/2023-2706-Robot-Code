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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ChargeCommand;
import frc.robot.commands.ChargeStationLock;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.GripperCommand.GRIPPER_INSTRUCTION;
import frc.robot.commands.SetBlingCommand;
import frc.robot.commands.TranslationCommand;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.config.Config;
import frc.robot.robotcontainers.CompRobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */

public class AutoRoutines {
    SwerveAutoBuilder autoBuilder;

    List<PathPlannerTrajectory> cube_0p5_top_charge_good;
    List<PathPlannerTrajectory> cube_0p5_bottom_charge;
    List<PathPlannerTrajectory> cube_0p5_middle_charge;
    List<PathPlannerTrajectory> cube_1p0_top;
    List<PathPlannerTrajectory> cube_1p0_bottom;
    List<PathPlannerTrajectory> cone_0p5_top_charge;
    List<PathPlannerTrajectory> cone_0p5_middle1_charge;    
    List<PathPlannerTrajectory> cone_0p5_middle2_charge;
    List<PathPlannerTrajectory> cone_0p5_bottom_charge;
    List<PathPlannerTrajectory> cone_1p0_top;
    List<PathPlannerTrajectory> cone_1p0_bottom;
    List<PathPlannerTrajectory> BK_cube_2p0_bottom;
    List<PathPlannerTrajectory> BK_cube_0p5_middle_charge;
    List<PathPlannerTrajectory> BK_cone_0p5_charge;
    List<PathPlannerTrajectory> place_pick_bottom2_charge_new;
    List<PathPlannerTrajectory> place_pick_place_pick_place_bottom2;
    List<PathPlannerTrajectory> place_pick_place_pick_place_bottom2_charge;
    List<PathPlannerTrajectory> place_pick_place_pick_place_bottom_new;
    List<PathPlannerTrajectory> cube_1p0_top_charge;

   
    public AutoRoutines() {
        Map<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("intake", new InstantCommand());
        eventMap.put("Bling Purple", new SetBlingCommand(1));
        eventMap.put("Bling Blue", new SetBlingCommand(2));
        eventMap.put("Bling Red", new SetBlingCommand(3));
        eventMap.put("Bling Honeydew", new SetBlingCommand(4));
        eventMap.put("charge", new TranslationCommand(-1.9, 0).andThen(
            Commands.run(() -> SwerveSubsystem.getInstance().setModuleStatesAuto(new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(-0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(-0.1, Rotation2d.fromDegrees(-45)),
            })).withTimeout(0.4)));

        eventMap.put("charge2", new ChargeCommand(-2.9).andThen(new ChargeStationLock()));

        //2.3
         
         //place game pieces
         eventMap.put("ArmCubeTop", new ArmCommand(ArmSetpoint.TOP_CUBE));
         eventMap.put("ArmCubeMiddle", new ArmCommand(ArmSetpoint.MIDDLE_CUBE));
         eventMap.put("ArmCubeBottom", new ArmCommand(ArmSetpoint.BOTTOM_CUBE));
         eventMap.put("ArmPickup", new ArmCommand(ArmSetpoint.PICKUP));
         eventMap.put("ArmHome", new ArmCommand(ArmSetpoint.HOME_WITH_GAMEPIECE));
         eventMap.put("ArmHomeAfterPickup", new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP));

         eventMap.put("GripperPickCube", new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CUBE, 
                                            CompRobotContainer.setState));
         eventMap.put("GripperPickCone", new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CONE, 
                                            CompRobotContainer.setState));

         eventMap.put("GripperOpen", new GripperCommand(GRIPPER_INSTRUCTION.OPEN, 
                                            CompRobotContainer.setState));

         
        autoBuilder = new SwerveAutoBuilder(
                SwerveSubsystem.getInstance()::getPose,
                SwerveSubsystem.getInstance()::resetOdometry,
                Config.Swerve.kSwerveDriveKinematics,
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0.2),
                SwerveSubsystem.getInstance()::setModuleStatesAuto,
                eventMap,
                true,
                SwerveSubsystem.getInstance());

        cube_0p5_top_charge_good = PathPlanner.loadPathGroup("cube_0p5_top_charge_goode", 2.5, 3);
        cube_0p5_bottom_charge = PathPlanner.loadPathGroup("cube_0p5_bottom_charge", 2.5, 3);
        cube_0p5_middle_charge = PathPlanner.loadPathGroup("cube_0p5_middle_charge", 2.5, 3);
        cube_1p0_top = PathPlanner.loadPathGroup("cube_1p0_top", 2.5, 3);
        cube_1p0_bottom = PathPlanner.loadPathGroup("cube_1p0_bottom", 2.5, 3);
        cone_0p5_top_charge = PathPlanner.loadPathGroup("cone_0p5_top_charge", 2.5, 3);
        cone_0p5_middle1_charge = PathPlanner.loadPathGroup("cone_0p5_middle1_charge", 2.5, 3);
        cone_0p5_middle2_charge = PathPlanner.loadPathGroup("cone_0p5_middle2_charge", 2.5, 3);
        cone_0p5_bottom_charge = PathPlanner.loadPathGroup("cone_0p5_bottom_charge", 2.5, 3);
        cone_1p0_top = PathPlanner.loadPathGroup("cone_1p0_top", 2.5, 3);
        cone_1p0_bottom = PathPlanner.loadPathGroup("cone_1p0_bottom", 2.5, 3);
        
        // HUMBER:
        cube_1p0_top_charge = PathPlanner.loadPathGroup("cube_1p0_top_charge", 2.5, 3);

        BK_cube_2p0_bottom = PathPlanner.loadPathGroup("BK_cube_2p0_bottom", 2.5, 3);
        BK_cube_0p5_middle_charge = PathPlanner.loadPathGroup("BK_cube_0p5_middle_charge", 2.5, 3);
        BK_cone_0p5_charge = PathPlanner.loadPathGroup("BK_cone_0p5_charge", 2.5, 3);
        
        place_pick_bottom2_charge_new = PathPlanner.loadPathGroup("place_pick_bottom2_charge_new", 2.5, 3);
        place_pick_place_pick_place_bottom2 = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom2", 2.5, 3);
        place_pick_place_pick_place_bottom2_charge = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom2_charge", 2.5, 3);
        place_pick_place_pick_place_bottom_new = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom_new", 2.5, 3);
    }

    public Command getAutonomousCommand(int selectAuto) {
        System.out.println (selectAuto);
        switch (selectAuto) {
            case 0:
                return null;

            case 1:
                return (autoBuilder.fullAuto(cube_0p5_top_charge_good));

            case 2:
                return (autoBuilder.fullAuto(cube_0p5_bottom_charge));

            case 3:
                return (autoBuilder.fullAuto(cube_0p5_middle_charge));

            case 4:
                return (autoBuilder.fullAuto(cube_1p0_top));
            
            case 5:
                return (autoBuilder.fullAuto(cube_1p0_bottom));

            case 6:
                return (autoBuilder.fullAuto(cone_0p5_top_charge));

            case 7:
                return (autoBuilder.fullAuto(cone_0p5_middle1_charge));

            case 8:
                return (autoBuilder.fullAuto(cone_0p5_middle2_charge));
                
            case 9:
                return (autoBuilder.fullAuto(cone_0p5_bottom_charge));

            case 10:
                return (autoBuilder.fullAuto(cone_1p0_top));
                
            case 11:
                return (autoBuilder.fullAuto(cone_1p0_bottom));
                

            case 12:
                return (autoBuilder.fullAuto(BK_cube_2p0_bottom));

            case 13:
                return (autoBuilder.fullAuto(BK_cube_0p5_middle_charge));

            case 14:
                return (autoBuilder.fullAuto(BK_cone_0p5_charge));

            case 15:
                return (autoBuilder.fullAuto(place_pick_bottom2_charge_new));
                
            case 16:
                return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom2));

            case 17:
                return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom2_charge));

            case 18:
                return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom_new));

            case 19:
                return (autoBuilder.fullAuto(cube_1p0_top_charge));

        }
        return new InstantCommand();
    }
}
