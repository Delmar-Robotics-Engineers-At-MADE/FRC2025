// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final PhotonVisionSensor m_photon = new PhotonVisionSensor();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_photon);

  // The driver's controller
  /* XboxController */ GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);

  // Operator's controller, for now the new PXN
  GenericHID m_buttonPad = new GenericHID(OIConstants.kButtonPadPort);

  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1)/4, OIConstants.kDriveDeadband), // getLeftY()
                MathUtil.applyDeadband(-m_driverController.getRawAxis(0)/4, OIConstants.kDriveDeadband), // getLeftX()
                -MathUtil.applyDeadband(m_driverController.getRawAxis(2)/4, OIConstants.kDriveDeadband*2), // getRightX()
                true),
            m_robotDrive));

    // register named commands for pathplanner
    NamedCommands.registerCommand("initiateX", m_robotDrive.setXCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));

    new JoystickButton(m_driverController, 1) // red B on logitech, trigger button on flight controller
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));

    new JoystickButton(m_driverController, 2) // thumb button on flight controller
        .whileTrue(new RunCommand(() -> m_robotDrive.resetOdometryToVision(m_photon), m_robotDrive, m_photon));

    new JoystickButton(m_buttonPad, 3) 
        .whileTrue(
            m_robotDrive.setTrajectoryToAprilTargetCmd(6, m_photon)
            .andThen(m_robotDrive.getSwerveControllerCmdForTeleop(m_photon))
            .andThen(() -> m_robotDrive.drive(0, 0, 0, false))
        );

    new JoystickButton(m_driverController, 4) // thumb button on flight controller
        .whileTrue(m_robotDrive.setXCommand());

  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond/4,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared/2)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Command resetPoseCommand = new InstantCommand(() -> 
    //     m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()));

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    Command myCmd = m_robotDrive.setTrajectoryToProcessorCmd(m_photon)
        .andThen(m_robotDrive.getSwerveControllerCmdForTeleop(m_photon))
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    return /* myCmd;*/ m_autoChooser.getSelected();
  }
}
