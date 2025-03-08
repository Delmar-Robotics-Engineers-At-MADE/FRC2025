// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  static final double TriggerThreshold = 0.5;

  // The robot's subsystems and commands are defined here...
  private final CoralSubsystem m_exampleSubsystem3 = new CoralSubsystem();

  // Driver
  GenericHID m_driverController = new GenericHID(0);
  CommandGenericHID m_driverCmdController = new CommandGenericHID (0);
  // Operator
  XboxController m_operController = new XboxController(1);
  CommandXboxController  m_operCmdController = new CommandXboxController (1);
  // Button pad (PXN)
  GenericHID m_buttonPad = new GenericHID(2);
  CommandGenericHID  m_buttonPadCmd = new CommandGenericHID (2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_operCmdController.x().onTrue(m_exampleSubsystem3.holdCurrentPositionCmd()
    //     .andThen(m_exampleSubsystem2.greenCmd()));

    // Manual control when Back or Start buttons are pressed
        
    m_operCmdController.leftBumper().and(() -> !m_exampleSubsystem3.getCoralPresent())
        .whileTrue(m_exampleSubsystem3.moveVelocityCommand(true));
    m_operCmdController.leftTrigger(TriggerThreshold)
        .whileTrue(m_exampleSubsystem3.moveVelocityCommand(false));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new WaitUntilCommand(5);
  }
}
