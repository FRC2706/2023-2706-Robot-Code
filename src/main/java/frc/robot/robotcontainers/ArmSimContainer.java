package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmAngle;

public class ArmSimContainer extends RobotContainer{
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    public ArmSimContainer() {
        configureButtonBindings();
    }
    public void configureButtonBindings() {

    m_driverController.a().whileTrue(new ArmAngle(50));
    m_driverController.b().whileTrue(new ArmAngle(90));
    m_driverController.x().whileTrue(new ArmAngle(140));
    m_driverController.y().whileTrue(new ArmAngle(170));
    }
}
