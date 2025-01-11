// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.RotateRadians;
import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XRPDrivetrain m_xrpDrivetrain = new XRPDrivetrain();

  // private final DriveDistance m_autoCommand = new DriveDistance(m_xrpDrivetrain, 12);
  // private final RotateRadians m_autoCommand = new RotateRadians(m_xrpDrivetrain, Math.PI / 4);

  private final Command m_autoCommand = new SequentialCommandGroup (
    new DriveDistance(m_xrpDrivetrain, 6),
    new WaitCommand(1),
    new RotateRadians(m_xrpDrivetrain, Math.PI / 4),
    new WaitCommand(1),
    new DriveDistance(m_xrpDrivetrain, 6),
    new WaitCommand(1),
    new RotateRadians(m_xrpDrivetrain, Math.PI / 4),
    new WaitCommand(1),
    new DriveDistance(m_xrpDrivetrain, 6),
    new WaitCommand(1),
    new RotateRadians(m_xrpDrivetrain, Math.PI / 4),
    new WaitCommand(1),
    new DriveDistance(m_xrpDrivetrain, 6),
    new WaitCommand(1),
    new RotateRadians(m_xrpDrivetrain, Math.PI / 4)
  );

  private XboxController driver = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_xrpDrivetrain.setDefaultCommand(
      new RunCommand(()-> m_xrpDrivetrain.arcadeDrive(-driver.getLeftY(), -driver.getRightX()), m_xrpDrivetrain)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
