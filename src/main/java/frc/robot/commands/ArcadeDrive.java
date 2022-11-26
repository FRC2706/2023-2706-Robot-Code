// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DiffNeoSubsystem;
import frc.robot.subsystems.DiffTalonSubsystem;

public class ArcadeDrive extends CommandBase {
    SubsystemBase m_subsystem;
    Supplier<Double> m_forward;
    Supplier<Double> m_steering;

    /**
     * Use the other constructor. Controls a Differential Drive robot in teleop.
     * 
     * This constructor will not handle the deadband or inverting the forward axis.
     * 
     * @param joystickForward Supplier to get the desired forward movement, range [-1, 1]
     * @param joystickSteering Supplier to get the desired steering movement, range [-1, 1]
     * @param subsystem The DriveSubsystem
     */
    public ArcadeDrive(Supplier<Double> joystickForward, Supplier<Double> joystickSteering) {
        m_forward = joystickForward;
        m_steering = joystickSteering;

        if (Config.DIFF.ISNEOS) {
            addRequirements(DiffNeoSubsystem.getInstance());
        } else {
            addRequirements(DiffTalonSubsystem.getInstance());
        }
    }

    /**
     * Prefered constructor. Controls a Differential Drive robot in teleop.
     * 
     * Inverts the forward axis.
     * Applys a deadband (value from Config.java)
     * 
     * @param driverStick Joystick of the driver's XboxController (or other controllers)
     * @param forwardAxis id of the axis for forwards
     * @param steeringAxis id of the axis for steering
     * @param subsystem The DriveSubsystem
     */
    public ArcadeDrive(Joystick driverStick, int forwardAxis, int steeringAxis) {
        this(
            () -> -1 * MathUtil.applyDeadband(driverStick.getRawAxis(forwardAxis), Config.DRIVER_JOYSTICK_DEADBAND),
            () -> MathUtil.applyDeadband(driverStick.getRawAxis(steeringAxis), Config.DRIVER_JOYSTICK_DEADBAND)
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Config.DIFF.ISNEOS) {
            DiffNeoSubsystem.getInstance().setIdleMode(Config.DIFF.TELEOP_IDLEMODE);
        } else {
            DiffTalonSubsystem.getInstance().setNeutralMode(Config.DIFF.TELEOP_NEUTRALMODE);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Config.DIFF.ISNEOS) {
            DiffNeoSubsystem.getInstance().arcadeDrive(m_forward.get(), m_steering.get());
        } else {
            DiffTalonSubsystem.getInstance().arcadeDrive(m_forward.get(), m_steering.get());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
