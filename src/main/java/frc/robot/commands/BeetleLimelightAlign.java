// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.libLimelight.LimelightHelpers;
import frc.robot.subsystems.DiffTalonSubsystem;

public class BeetleLimelightAlign extends CommandBase {
    private final String LIMELIGHT_NAME = "limelight";
    private final double FLIP_LIMELIGHT_TX = 1;

    private final double desiredHeadingDeg = 0; // deg
    private final double desiredArea = 1.423569;

    private final PIDController rotationController = new PIDController(0.04, 0.005, 0);

    private final double FORWARD_kF = 0.01;
    // private final PIDController forwardController = new PIDController(1, 0, 0);
    private final ProfiledPIDController forwardController = new ProfiledPIDController(1.6, 0.01, 0, new Constraints(0.6, 4));

    // private final SlewRateLimiter forwardAccelLimiter = new SlewRateLimiter(2, -999, 0);

    /** Creates a new BeetleLimelightAlign. */
    public BeetleLimelightAlign() {
        addRequirements(DiffTalonSubsystem.getInstance());

        rotationController.setIntegratorRange(-0.25, 0.25);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 8);

        // forwardAccelLimiter.reset(0);
        forwardController.reset(LimelightHelpers.getTA(LIMELIGHT_NAME), DiffTalonSubsystem.getInstance().getAveragePercentOutput());
        // forwardController.reset(LimelightHelpers.getTA(LIMELIGHT_NAME), 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX(LIMELIGHT_NAME) * FLIP_LIMELIGHT_TX;
        double ta = LimelightHelpers.getTA(LIMELIGHT_NAME);

        double rotVal = rotationController.calculate(tx, desiredHeadingDeg);

        // double forVal = forwardAccelLimiter.calculate(forwardController.calculate(ta, desiredArea)) 
        //                         + Math.copySign(FORWARD_kF, desiredArea - ta);

        double forVal = forwardController.calculate(ta, desiredArea)
                                + Math.copySign(FORWARD_kF, desiredArea - ta);

        DiffTalonSubsystem.getInstance().arcadeDrive(forVal, rotVal);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        DiffTalonSubsystem.getInstance().stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
