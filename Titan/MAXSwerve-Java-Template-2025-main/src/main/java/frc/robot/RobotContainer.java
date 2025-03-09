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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetBlinkinColorCmd;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.HornSelection;
import frc.robot.subsystems.DriveSubsystem.HornSelection.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSensor;
import frc.robot.subsystems.Blinkin.LEDConstants;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  static final double TriggerThreshold = 0.5;

  // The robot's subsystems
  private final PhotonVisionSensor m_photon = new PhotonVisionSensor();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_photon);
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final Blinkin m_blinkin = new Blinkin();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();

  // for auto driving
//   Alliance m_allianceColor = DriverStation.getAlliance().get();
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  Command m_autoToReef1LToCoralStation, m_autoToReef1RToCoralStation;
  Command m_autoToReef2LToCoralStation, m_autoToReef2RToCoralStation;
  Command m_autoToReef4LToCoralStation, m_autoToReef4RToCoralStation;

  // Driver
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  CommandGenericHID m_driverCmdController = new CommandGenericHID (OIConstants.kDriverControllerPort);
  // Operator
  XboxController m_operController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController  m_operCmdController = new CommandXboxController (OIConstants.kOperatorControllerPort);
  // Button pad (PXN)
  GenericHID m_buttonPad = new GenericHID(OIConstants.kButtonPadPort);
  CommandGenericHID  m_buttonPadCmd = new CommandGenericHID (OIConstants.kButtonPadPort);

  // private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure non-button triggers
    configureNonButtonTriggers();

    // setup dashboard
    setupDashboard();

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
    // NamedCommands.registerCommand("initiateX", m_robotDrive.setXCommand());
  }

  private Command driveToAprilTagCommand (int id, HornSelection hornSelect) {
    // System.out.println("New command to tag " + id);
    return m_robotDrive.setTrajectoryToAprilTargetCmd(id, hornSelect, m_photon)
    .andThen(m_robotDrive.getSwerveControllerCmdForTeleop(m_photon))
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  int[] redReefPositionToAprilTag = {0, 11, 10, 9, 6, 7, 8};
  // int[] blueReefPositionToAprilTag = {0, 20, 21, 22, 19, 18, 17};
  private Command driveToReefPositionCmd (int pos, HornSelection hornSelect) {
    // if (m_allianceColor == Alliance.Red) {
        return driveToAprilTagCommand (redReefPositionToAprilTag[pos], hornSelect);
    // } else {
    //     return driveToAprilTagCommand (blueReefPositionToAprilTag[pos], hornSelect);
    // }
  }

  private void buildAutoChooser() {
    m_autoToReef1LToCoralStation = new WaitUntilCommand(() -> m_photon.getPoseEstimateAcquired())
        .andThen(driveToReefPositionCmd(1, HornSelection.L))
        .andThen(new SetBlinkinColorCmd(m_blinkin, LEDConstants.green));
    m_autoToReef1RToCoralStation = driveToReefPositionCmd(1, HornSelection.R);
    m_autoToReef2LToCoralStation = driveToReefPositionCmd(2, HornSelection.L);
    m_autoToReef2RToCoralStation = driveToReefPositionCmd(2, HornSelection.R);
    m_autoToReef4LToCoralStation = driveToReefPositionCmd(4, HornSelection.L);
    m_autoToReef4RToCoralStation = driveToReefPositionCmd(4, HornSelection.R);
    m_autoChooser.setDefaultOption("To Reef 1L", m_autoToReef1LToCoralStation);
    m_autoChooser.addOption("To Reef 1R", m_autoToReef1RToCoralStation);
    m_autoChooser.addOption("To Reef 2L", m_autoToReef2LToCoralStation);
    m_autoChooser.addOption("To Reef 2R", m_autoToReef2RToCoralStation);
    m_autoChooser.addOption("To Reef 4L", m_autoToReef4LToCoralStation);
    m_autoChooser.addOption("To Reef 4R", m_autoToReef4RToCoralStation);
  }

  private void configureNonButtonTriggers() {
    new Trigger(() -> m_coral.getCoralPresent())
        .whileTrue(new SetBlinkinColorCmd(m_blinkin, LEDConstants.grey));
    new Trigger(() -> m_algae.getAlgaePresent())
        .whileTrue(new SetBlinkinColorCmd(m_blinkin, LEDConstants.green));
  }

  private void configureButtonBindings() {

    // *************************** DRIVER *****************************

    new JoystickButton(m_driverController, 2) // thumb button on flight controller
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));

    // reef positions
    m_driverCmdController.button(3).and(m_buttonPadCmd.button(3)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(1, HornSelection.L));
    m_driverCmdController.button(4).and(m_buttonPadCmd.button(3)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(1, HornSelection.R));
    m_driverCmdController.povUp().and(m_buttonPadCmd.button(3)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(1, HornSelection.Between));
    m_driverCmdController.button(3).and(m_buttonPadCmd.button(4)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(2, HornSelection.L));
    m_driverCmdController.button(4).and(m_buttonPadCmd.button(4)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(2, HornSelection.R));
    m_driverCmdController.button(3).and(m_buttonPadCmd.button(6)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(3, HornSelection.L));
    m_driverCmdController.button(4).and(m_buttonPadCmd.button(6)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(3, HornSelection.R));
    m_driverCmdController.button(3).and(m_buttonPadCmd.button(1)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(4, HornSelection.L));
    m_driverCmdController.button(4).and(m_buttonPadCmd.button(1)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(4, HornSelection.R));
        m_driverCmdController.button(3).and(m_buttonPadCmd.button(2)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(5, HornSelection.L));
    m_driverCmdController.button(4).and(m_buttonPadCmd.button(2)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(5, HornSelection.R));
    m_driverCmdController.button(3).and(m_buttonPadCmd.axisGreaterThan(3,TriggerThreshold)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(6, HornSelection.L));
    m_driverCmdController.button(4).and(m_buttonPadCmd.axisGreaterThan(3,TriggerThreshold)).and(m_photon::getPoseEstimateAcquired)
        .whileTrue(driveToReefPositionCmd(6, HornSelection.R));

    // Coral stations and Processor

    // intake algae
    m_driverCmdController.button(1)
        .whileTrue(new WaitUntilCommand(() -> !m_algae.getAlgaePresent())
        .andThen(m_algae.moveVelocityCommand(true)));

    // set X
    m_driverCmdController.povUp().whileTrue(m_robotDrive.setXCommand());

    // ******************************** OPERATOR *********************************

    // coral in/out
    m_operCmdController.leftBumper().and(() -> !m_coral.getCoralPresent())
        .whileTrue(m_coral.moveVelocityCommand(true));
    m_operCmdController.leftTrigger(TriggerThreshold)
        .whileTrue(m_coral.moveVelocityCommand(false));

    // algae in/out
    m_operCmdController.a().and(m_operCmdController.povLeft()) // spit algae front
        .whileTrue(new WaitUntilCommand(() -> m_algae.getAlgaePresent())
        .andThen(m_algae.moveVelocityCommand(false)));

    m_operCmdController.a().and(m_operCmdController.povDown()) // spit algae rear
        .whileTrue(new WaitUntilCommand(() -> m_algae.getAlgaePresent())
        .andThen(m_algae.moveVelocityCommand(true)));

    m_operCmdController.a().and(m_operCmdController.povUp()) // shoot algae
        .whileTrue(new WaitUntilCommand(() -> m_algae.getAlgaePresent())
        .andThen(m_algae.moveVelocityCommand(false)));

    // Manual control when Back or Start buttons are pressed
    m_operCmdController.back().or(m_operCmdController.start())
        .and(m_operCmdController.leftBumper())
        .whileTrue(m_elevator.moveVelocityCommand(true, true, false));
    m_operCmdController.back().or(m_operCmdController.start())
        .and(m_operCmdController.rightBumper())
        .whileTrue(m_elevator.moveVelocityCommand(true, false, true));
    m_operCmdController.back().or(m_operCmdController.start())
        .and(m_operCmdController.leftTrigger(TriggerThreshold))
        .whileTrue(m_elevator.moveVelocityCommand(false, true, false));
    m_operCmdController.back().or(m_operCmdController.start())
        .and(m_operCmdController.rightTrigger(TriggerThreshold))
        .whileTrue(m_elevator.moveVelocityCommand(false, false, true));

    // reset pose to vision
    m_operCmdController.back().or(m_operCmdController.start())
        .and(m_operCmdController.y())
        .onTrue(new InstantCommand (() -> m_robotDrive.debugResetOdometryToVision(m_photon), m_robotDrive, m_photon));


  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond/4,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared/2)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);
    
    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // // Command resetPoseCommand = new InstantCommand(() -> 
    // //     m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()));

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // Command myCmd = m_robotDrive.setTrajectoryToAprilTargetCmd(6, false, false, m_photon)
    //     .andThen(m_robotDrive.getSwerveControllerCmdForTeleop(m_photon))
    //     .andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    return m_autoChooser.getSelected();
  }

  private void setupDashboard() {
    ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    matchTab.addCamera("Limelight", "Limelight", "http://10.80.77.18:5800");
    buildAutoChooser();
    matchTab.add(m_autoChooser);
  }
}
