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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToGamePiece;
import frc.robot.commands.AlignToTargetVision;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmJoystickConeCommand;
import frc.robot.commands.ChargeCommand;
import frc.robot.commands.ChargeCommandPigeon;
import frc.robot.commands.ChargeCommandPigeonExtend;
import frc.robot.commands.ChargeCommandRoll;
import frc.robot.commands.ChargeStationLock;
import frc.robot.commands.ForceBrakesInAuto;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.GripperCommand.GRIPPER_INSTRUCTION;
import frc.robot.commands.SetBlingCommand;
import frc.robot.commands.TranslationCommand;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.config.Config;
import frc.robot.robotcontainers.CompRobotContainer;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */

public class AutoRoutines {
    SwerveAutoBuilder autoBuilder;

    // HUMBER MAINS
    List<PathPlannerTrajectory> cube_0p5_top_charge;
    List<PathPlannerTrajectory> cube_0p5_bottom_charge;
    List<PathPlannerTrajectory> cube_0p5_middle_charge;
    List<PathPlannerTrajectory> cube_0p5_middle_charge_fast;
    List<PathPlannerTrajectory> cube_0p5_middle2_charge;
    List<PathPlannerTrajectory> cube_1p0_top;
    List<PathPlannerTrajectory> cube_1p0_bottom;
    List<PathPlannerTrajectory> cone_0p5_top_charge;
    List<PathPlannerTrajectory> cube_0p5_bottom;

    List<PathPlannerTrajectory> cube_3p0_top;


    //for tuning only
    List<PathPlannerTrajectory> forward;
    List<PathPlannerTrajectory> curve;

    
    List<PathPlannerTrajectory> cone_2p0_bot_P1;
    List<PathPlannerTrajectory> cone_2p0_bot_P2;

    // List<PathPlannerTrajectory> place_pick_place_pick_place_bottom2;

    // List<PathPlannerTrajectory> cube_0p5_top_charge_good;
    // List<PathPlannerTrajectory> cone_0p5_middle1_charge;    
    // List<PathPlannerTrajectory> cone_0p5_middle2_charge;
    // List<PathPlannerTrajectory> cone_0p5_bottom_charge;
    // List<PathPlannerTrajectory> cone_1p0_top;
    // List<PathPlannerTrajectory> cone_1p0_bottom;
    // List<PathPlannerTrajectory> BK_cube_2p0_bottom;
    // List<PathPlannerTrajectory> BK_cube_0p5_middle_charge;
    // List<PathPlannerTrajectory> BK_cone_0p5_charge;
    // List<PathPlannerTrajectory> place_pick_bottom2_charge_new;
    // List<PathPlannerTrajectory> place_pick_place_pick_place_bottom2_charge;
    // List<PathPlannerTrajectory> place_pick_place_pick_place_bottom_new;

   
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

        eventMap.put("charge2", new ChargeCommand(-3.05).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeStationLock()));
        eventMap.put("charge3", new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommand(4.1).andThen(new ChargeStationLock())));
        eventMap.put("chargeRoll", new WaitCommand(0.4).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommandRoll(3.2).alongWith(new ArmCommand(ArmSetpoint.HOME_WITH_GAMEPIECE))).andThen(new ChargeStationLock()));
        
        //wide and positive X flip
        Command chargePigeonWideNxLowerArm = new ArmCommand(ArmSetpoint.HOME_WITH_GAMEPIECE).asProxy();
        Command chargePigeonWideNxCommand = new SequentialCommandGroup(
            new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))),
            new ChargeCommandPigeon(true, 1.0).alongWith(new WaitCommand(0.2).andThen(Commands.runOnce(() -> chargePigeonWideNxLowerArm.schedule()))),
            new ChargeStationLock().withTimeout(1),
            Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))
        );
        
        eventMap.put("chargePigeonWidePx", chargePigeonWideNxCommand);// new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommandPigeon(true, 1.0).alongWith(Commands.runOnce(() -> chargePigeonWideNxLowerArm.schedule()))).andThen(new ChargeStationLock()).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))));
        
        Command chargePigeonExtendCommand = new SequentialCommandGroup(
            new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))),
            new ChargeCommandPigeonExtend().alongWith(new WaitCommand(0.2).andThen(Commands.runOnce(() -> chargePigeonWideNxLowerArm.schedule()))),
            new ChargeStationLock().withTimeout(1),
            Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))
        );
        eventMap.put("chargePigeonExtend", chargePigeonExtendCommand);// new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommandPigeon(true, 1.0).alongWith(Commands.runOnce(() -> chargePigeonWideNxLowerArm.schedule()))).andThen(new ChargeStationLock()).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).andThen(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))));
        
        
        eventMap.put("chargePigeonWideNx", new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommandPigeon(true, -1.0).andThen(new ChargeStationLock())));
        eventMap.put("chargePigeonNarrowPx", new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommandPigeon(false, 1.0).andThen(new ChargeStationLock())));
        eventMap.put("chargePigeonNarrowNx", new WaitCommand(0.3).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true)))).andThen(new ChargeCommandPigeon(false, -1.0).andThen(new ChargeStationLock())));

        //2.3
         
         //place game pieces
         eventMap.put("ArmCubeTop", new ArmCommand(ArmSetpoint.TOP_CUBE).withTimeout(4.5).andThen((Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true))))));
         eventMap.put("ArmCubeTopProxy", new ArmCommand(ArmSetpoint.TOP_CUBE).withTimeout(4.5).asProxy().andThen((Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(true)).alongWith(Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(true))))));
         eventMap.put("ArmCubeTopNoWP", new ArmCommand(ArmSetpoint.TOP_CONE_NO_WAYPOINT));
         eventMap.put("ArmCubeMiddle", new ArmCommand(ArmSetpoint.MIDDLE_CUBE));
         eventMap.put("ArmCubeBottom", new ArmCommand(ArmSetpoint.BOTTOM_CUBE));
         eventMap.put("ArmPickup", new ArmCommand(ArmSetpoint.PICKUP));
         eventMap.put("ArmPickupNoWP", new ArmCommand(ArmSetpoint.PICKUP_NOWP));
         eventMap.put("ArmHome", new ArmCommand(ArmSetpoint.HOME_WITH_GAMEPIECE));
         eventMap.put("ArmHomeAfterPickup", new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP));

         eventMap.put("Arm2CubeTop", schedule(new ArmCommand(ArmSetpoint.TOP_CUBE)));
         eventMap.put("Arm2ConeTop", schedule(new ArmJoystickConeCommand(ArmSetpoint.TOP_CONE, new CommandXboxController(3))));
         eventMap.put("Arm2Pickup", schedule(new ArmCommand(ArmSetpoint.PICKUP)));
         eventMap.put("Arm2HomeAfterPickup", schedule(new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP)));

         eventMap.put("Arm2TopConeLowerThenScore", schedule(new ParallelCommandGroup(
            new ArmCommand(ArmSetpoint.TOP_CONE_RELEASE),
            new WaitCommand(0.3).andThen(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, CompRobotContainer.setState))
        )));

        eventMap.put("SetStateBaseCone", Commands.runOnce(() -> CompRobotContainer.setRobotGamePieceState(RobotGamePieceState.HasBaseCone)));

         eventMap.put("GripperPickCube", new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CUBE, 
                                            CompRobotContainer.setState));
         eventMap.put("GripperPickCone", new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CONE, 
                                            CompRobotContainer.setState));

        eventMap.put("Gripper2PickCone", schedule(new SequentialCommandGroup(
            new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CONE, CompRobotContainer.setState),
            new WaitCommand(0.4),
            new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP))));

        eventMap.put("Gripper2SetConeStart", schedule(new SequentialCommandGroup(
            new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CONE, CompRobotContainer.setState),
            Commands.runOnce(() -> CompRobotContainer.setRobotGamePieceState(RobotGamePieceState.HasBaseCone)))
        ));

        eventMap.put("Gripper2PickCube", schedule(new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CUBE, 
                                CompRobotContainer.setState)));

         eventMap.put("GripperOpen", new GripperCommand(GRIPPER_INSTRUCTION.OPEN, 
                                            CompRobotContainer.setState));

        eventMap.put("Gripper2Open", schedule(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, 
                                            CompRobotContainer.setState)));

        eventMap.put("wait", new WaitCommand(0.3));
         
        autoBuilder = new SwerveAutoBuilder(
                SwerveSubsystem.getInstance()::getPose,
                SwerveSubsystem.getInstance()::resetOdometry,
                Config.Swerve.kSwerveDriveKinematics,
                new PIDConstants(7, 0, 0), // Translation PID
                new PIDConstants(9, 0, 0.2), // Heading PID
                SwerveSubsystem.getInstance()::setModuleStatesAuto,
                eventMap,
                true,
                SwerveSubsystem.getInstance());

        // HUMBER MAINS
        cube_0p5_top_charge = PathPlanner.loadPathGroup("cube_0p5_top_charge", 2.5, 3);
        cube_0p5_bottom_charge = PathPlanner.loadPathGroup("cube_0p5_bottom_charge", 2.5, 3);
        cube_0p5_middle_charge = PathPlanner.loadPathGroup("cube_0p5_middle_charge", 2.5, 3);
        cube_0p5_middle_charge_fast = PathPlanner.loadPathGroup("cube_0p5_middle_charge_fast", 2.5, 3);
        cube_0p5_middle2_charge = PathPlanner.loadPathGroup("cube_0p5_middle2_charge", 2.5, 3);
        cube_1p0_top = PathPlanner.loadPathGroup("cube_1p0_top", 2.5, 3);
        cube_1p0_bottom = PathPlanner.loadPathGroup("cube_1p0_bottom", 2.5, 3);
        cube_0p5_bottom = PathPlanner.loadPathGroup("cube_0p5_bottom", 2.5, 3);
        cube_3p0_top = PathPlanner.loadPathGroup("cube_3p0_top", 2.6, 3.1);

        cone_0p5_top_charge = PathPlanner.loadPathGroup("cone_0p5_top_charge", 2.5, 3);

        cone_2p0_bot_P1 = PathPlanner.loadPathGroup("cone_2p0_bot_P1", 2.7, 3.8);
        cone_2p0_bot_P2 = PathPlanner.loadPathGroup("cone_2p0_bot_P2", 2.7, 3.8);

        forward = PathPlanner.loadPathGroup("TuningDriveForward", 2.5, 3);
        curve = PathPlanner.loadPathGroup("TuningDriveCurve", 2.5, 3);

        // Possible Humber
        // cone_2p0_bot = PathPlanner.loadPathGroup("cone_2p0_bot", 2.5, 3);
        // place_pick_place_pick_place_bottom2 = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom2", 2.5, 3);

        // cube_0p5_top_charge_good = PathPlanner.loadPathGroup("cube_0p5_top_charge_good", 2.5, 3);
        // cone_0p5_middle1_charge = PathPlanner.loadPathGroup("cone_0p5_middle1_charge", 2.5, 3);
        // cone_0p5_middle2_charge = PathPlanner.loadPathGroup("cone_0p5_middle2_charge", 2.5, 3);
        // cone_0p5_bottom_charge = PathPlanner.loadPathGroup("cone_0p5_bottom_charge", 2.5, 3);
        // cone_1p0_top = PathPlanner.loadPathGroup("cone_1p0_top", 2.5, 3);
        // cone_1p0_bottom = PathPlanner.loadPathGroup("cone_1p0_bottom", 2.5, 3);       

        // BK_cube_2p0_bottom = PathPlanner.loadPathGroup("BK_cube_2p0_bottom", 2.5, 3);
        // BK_cube_0p5_middle_charge = PathPlanner.loadPathGroup("BK_cube_0p5_middle_charge", 2.5, 3);
        // BK_cone_0p5_charge = PathPlanner.loadPathGroup("BK_cone_0p5_charge", 2.5, 3);
        
        // place_pick_bottom2_charge_new = PathPlanner.loadPathGroup("place_pick_bottom2_charge_new", 2.5, 3);
        // place_pick_place_pick_place_bottom2_charge = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom2_charge", 2.5, 3);
        // place_pick_place_pick_place_bottom_new = PathPlanner.loadPathGroup("place_pick_place_pick_place_bottom_new", 2.5, 3);
    }

    public Command getAutonomousCommand(int selectAuto) {
        System.out.println (selectAuto);
        switch (selectAuto) {
            case 0:
                return new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CUBE, CompRobotContainer.setState).andThen(Commands.runOnce(()-> SwerveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)))));

            case 1:
                return (autoBuilder.fullAuto(cube_0p5_top_charge)); 
                // return(autoBuilder.fullAuto(forward));
             
            case 2:
                DoubleSubscriber yawSub = NetworkTableInstance.getDefault().getTable("MergeVisionPipelineIntake22").getDoubleTopic("Yaw").subscribe(-99);
                return new SequentialCommandGroup(
                    schedule(new ArmJoystickConeCommand(ArmSetpoint.TOP_CONE, new CommandXboxController(3))), // Starting lifting the arm to the top cone node
                    new WaitCommand(0.35), // Allow the arm time to lift the arm.
                    Commands.runOnce(() -> SwerveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)))), // Reset heading for next command
                    new AlignToTargetVision(true, 1.0, 0.05, 0, Math.PI, 2.5, 2.0, true).withTimeout(2.4), // Use vision to align with tape target
                    schedule(new ParallelCommandGroup(
                        new ArmCommand(ArmSetpoint.TOP_CONE_RELEASE), // Lower the arm
                        new WaitCommand(0.2).andThen(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, CompRobotContainer.setState)) // Then release the gripper
                    )).withTimeout(0.1),
                    new WaitCommand(0.25), // Allow time to lower and release game piece
                    autoBuilder.fullAuto(cone_2p0_bot_P1), // Run P1 PathPlanner. Will reset odometry to where vision aligned the robot.
                    new AlignToGamePiece(yawSub, 1.2), // Use vision to align to game piece. Increase value here (currently 1.2 meters) if the robot doesn't drive far enough to grab the cone.
                    schedule(new SequentialCommandGroup(
                        new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CONE, CompRobotContainer.setState), // Pick up the cone
                        new WaitCommand(0.3), // Give time for the cone is be grabbed aka pistons to extend
                        new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP))), // Lift arm to home position
                    new WaitCommand(0.5), // Give time for cone to be grabbed and arm lifted
                    autoBuilder.followPathGroupWithEvents(cone_2p0_bot_P2), // Run P2 PathPlanner. Will not reset odometry.
                    new AlignToTargetVision(true, 1.0, 0.05, 0, Math.PI, 2.5, 2.5, true).withTimeout(3), // Use vision to align to tape node target.
                    schedule(new ParallelCommandGroup(
                        new ArmCommand(ArmSetpoint.TOP_CONE_RELEASE), // Lower arm
                        new WaitCommand(0.2).andThen(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, CompRobotContainer.setState)))), // Release gripper
                    new WaitCommand(3) // Give time to lower and release cone
                ).withTimeout(14.95).andThen(brakes(true)); // Force auto to end at 14.95 seconds and ensure the brakes are on
         
            case 3:
                // return (autoBuilder.fullAuto(cube_0p5_middle_charge)); // This slower one should still work

                Command lowerArm = new ArmCommand(ArmSetpoint.HOME_WITH_GAMEPIECE).asProxy();
                return new SequentialCommandGroup(
                    autoBuilder.fullAuto(cube_0p5_middle_charge_fast),
                    brakes(true),
                    new ParallelCommandGroup(
                        // new ChargeCommandPigeon(true, 1.0),  // Use either ChargeCommandPigeon or ChargeCommandPigeonExtend
                        new ChargeCommandPigeonExtend(), 
                        new WaitCommand(0.2).andThen(schedule(lowerArm))
                    ),
                    brakes(true)
                );
        
            case 4:
                return (autoBuilder.fullAuto(cube_1p0_top));
            
            case 5:
                return (autoBuilder.fullAuto(cube_0p5_bottom));

             case 6:
                return null;
             case 7:
             //Adding 7 here, since Analog Selector misses 6, from 5 to 7 directly. 
             // Drive team calls this 6 since they have to do 6 clicks on the switch.
                return Commands.runOnce(()-> SwerveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)))).andThen(
                 new ParallelCommandGroup(
                    new ChargeCommand(2.6),
                    new GripperCommand(GRIPPER_INSTRUCTION.PICK_UP_CUBE, CompRobotContainer.setState).withTimeout(1),
                    new ArmCommand(ArmSetpoint.HOME_AFTER_PICKUP).withTimeout(1)
                ).andThen(new ChargeStationLock()));

            // case 8:
                
                
            // case 9:
            //     return (autoBuilder.fullAuto(cone_0p5_bottom_charge));

            // case 10:
            //     return (autoBuilder.fullAuto(cone_1p0_top));
                
            // case 11:
            //     return (autoBuilder.fullAuto(cone_1p0_bottom));
                

            // case 12:
            //     return (autoBuilder.fullAuto(BK_cube_2p0_bottom));

            // case 13:
            //     return (autoBuilder.fullAuto(BK_cube_0p5_middle_charge));

            // case 14:
            //     return (autoBuilder.fullAuto(BK_cone_0p5_charge));

            // case 15:
            //     return (autoBuilder.fullAuto(place_pick_bottom2_charge_new));
                
            // case 16:
            //     return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom2));

            // case 17:
            //     return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom2_charge));

            // case 18:
            //     return (autoBuilder.fullAuto(place_pick_place_pick_place_bottom_new));

            // case 19:
            //     return (autoBuilder.fullAuto(cube_1p0_top_charge));

            // case 20:
            //     return new SequentialCommandGroup(
            //         autoBuilder.fullAuto(cone_2p0_bot),
            //         // new ArmCommand(ArmSetpoint.TOP_CONE_NO_WAYPOINT),
            //         new AlignToTargetVision(true, 1.0, 0.03, 0, Math.PI, 1.5, 1.7).withTimeout(1.3),
            //         new GripperCommand(GRIPPER_INSTRUCTION.OPEN, CompRobotContainer.setState)
            //     );

        }
        return new InstantCommand();
    }

    private Command schedule(Command command) {
        return Commands.runOnce(() -> command.schedule());
    }

    private Command brakes(boolean turnBrakeOn) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(turnBrakeOn)),
            Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(turnBrakeOn))
        );
    }
}
