// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class TestChargeStationFeedforward extends SwerveTeleop {


    DoubleEntry m_speedMultipler;

    CommandXboxController m_driver;

    double startingAngle = 0;

    /** Creates a new TestChargeStationFeedforward. */
    public TestChargeStationFeedforward(CommandXboxController driver)  {
        super(driver);
        m_driver = driver;

        m_speedMultipler = NetworkTableInstance.getDefault().getDoubleTopic("Drivetrain/ChargeFFMultiplier")
                                               .getEntry(Config.Swerve.chargeStationFFMultiplier);

        if (m_speedMultipler.exists() == false) {
            m_speedMultipler.accept(Config.Swerve.chargeStationFFMultiplier);
        }

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingAngle = SwerveSubsystem.getInstance().getPitch();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SwerveSubsystem.getInstance().drive(
            calculateX(),
            super.calculateY(),
            super.calculateRot(),
            true, false);
    }

    @Override
    public double calculateX() {
        double x = -1 * m_driver.getRawAxis(XboxController.Axis.kLeftY.value);
        x = MathUtil.applyDeadband(x, Config.DRIVER_JOYSTICK_DEADBAND);
        x *= Config.Swerve.teleopSpeed;

        double pitch = SwerveSubsystem.getInstance().getPitch() - startingAngle;
        double roll = SwerveSubsystem.getInstance().getRoll();

        double FF = m_speedMultipler.getAsDouble() * Math.cos(pitch);

        return x + FF;
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
