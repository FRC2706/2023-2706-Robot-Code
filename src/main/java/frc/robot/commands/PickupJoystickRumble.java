// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.ArmConfig;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.subsystems.ArmSubsystem;

public class PickupJoystickRumble extends CommandBase {
    private final double allowableBotErrorRad = Math.toRadians(3);
    private final double allowableTopErrorRad = Math.toRadians(3);
    private final double topLowerBounds;
    private final double topUpperBounds;
    private final double botLowerBounds;
    private final double botUpperBounds;

    private CommandXboxController m_operator;

    private CommandBase rumbleCommand;

    /** Creates a new ArmRumble. */
    public PickupJoystickRumble(CommandXboxController operator) {
        m_operator = operator;

        rumbleCommand = new StartEndCommand(
            () -> m_operator.getHID().setRumble(RumbleType.kBothRumble, 0.3),
            () -> m_operator.getHID().setRumble(RumbleType.kBothRumble, 0)
        ).withTimeout(0.2).ignoringDisable(true);

        double[] pickupAngles = ArmSubsystem.getInstance().inverseKinematics(
            ArmConfig.L1, ArmConfig.L2, ArmSetpoint.PICKUP.getX(), ArmSetpoint.PICKUP.getZ());

        botLowerBounds = pickupAngles[0] - allowableBotErrorRad;
        botUpperBounds = pickupAngles[0] + allowableBotErrorRad;

        topLowerBounds = pickupAngles[1] - allowableTopErrorRad;
        topUpperBounds = pickupAngles[1] + allowableTopErrorRad;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted == false) {
            rumbleCommand.schedule();
        }   
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double topPos = ArmSubsystem.getInstance().getTopPosition();
        double botPos = ArmSubsystem.getInstance().getBottomPosition();

        return topPos > topLowerBounds && topPos < topUpperBounds &&
               botPos > botLowerBounds && botPos < botUpperBounds;
    }
}
