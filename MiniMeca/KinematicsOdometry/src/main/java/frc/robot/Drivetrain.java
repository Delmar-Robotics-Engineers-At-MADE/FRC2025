// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;

class myAHRS extends AHRS {
  @Override
  public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
  }

  public myAHRS(SPI.Port kmxp, byte update_rate_hz) {
     super(kmxp, update_rate_hz);
  }
}

class DriveConstants {
  public static final int kEncoderCPR = 74;
  public static final double kWheelDiameterMeters = 0.10;
  public static final double kEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
}

/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 2* Math.PI; // 2 rotation per second

  private final Talon m_frontLeftMotor = new Talon(1);
  private final Talon m_frontRightMotor = new Talon(3);
  private final Talon m_backLeftMotor = new Talon(0);
  private final Talon m_backRightMotor = new Talon(2);

  private final Encoder m_frontLeftEncoder = new Encoder(2, 3);
  private final Encoder m_frontRightEncoder = new Encoder(6, 7);
  private final Encoder m_backLeftEncoder = new Encoder(0, 1);
  private final Encoder m_backRightEncoder = new Encoder(4, 5);

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.1588, 0.1651);
  // private final Translation2d m_frontRightLocation = new Translation2d(0.1588, -0.1651);
  // private final Translation2d m_backLeftLocation = new Translation2d(-0.1588, 0.1651);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.1588, -0.1651);
  private final Translation2d m_frontLeftLocation = new Translation2d(-0.1588, 0.1651);
  private final Translation2d m_frontRightLocation = new Translation2d(0.1588, 0.1651);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.1588, -0.1651);
  private final Translation2d m_backRightLocation = new Translation2d(0.1588, -0.1651);

  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

  private final myAHRS m_gyro = new myAHRS(SPI.Port.kMXP, (byte) 200);
  ShuffleboardTab m_driveBaseTab;
  ShuffleboardLayout m_speedsLayout, m_outputsLayout;

  double m_frontLeftOutput, m_frontRightOutput, m_backLeftOutput, m_backRightOutput;
  // double m_frontLeftSpeed, m_frontRightSpeed, m_backLeftSpeed, m_backRightSpeed;
  MecanumDriveWheelSpeeds m_speeds = new MecanumDriveWheelSpeeds(0,0,0,0);
  double m_rotationRadsPerSec;

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(m_kinematics, m_gyro.getRotation2d(), getCurrentDistances());

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(3, 6);

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    m_gyro.reset();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_backRightMotor.setInverted(true);
    m_backLeftMotor.setInverted(true);

    m_backRightEncoder.setReverseDirection(true);
    m_frontRightEncoder.setReverseDirection(true);

    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_backLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_backRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);


    m_driveBaseTab = Shuffleboard.getTab("Drivebase");
    m_driveBaseTab.add("Gyro", m_gyro);
    // m_driveBaseTab.add("Speeds", m_speeds);

    // Put both encoders in a list layout
    ShuffleboardLayout encodersLayout =
        m_driveBaseTab.getLayout("Encoders", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    encodersLayout.add("Front Left Encoder", m_frontLeftEncoder);
    encodersLayout.add("Front Right Encoder", m_frontRightEncoder);
    encodersLayout.add("Rear Left Encoder", m_backLeftEncoder);
    encodersLayout.add("Rear Right Encoder", m_backRightEncoder);    

    // m_outputsLayout =
    //     m_driveBaseTab.getLayout("Outputs", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 4);
    // encodersLayout.add("Front Left Output", () -> m_frontLeftOutput);
    // encodersLayout.add("Front Right Output", m_frontRightOutput);
    // encodersLayout.add("Rear Left Output", m_backLeftOutput);
    // encodersLayout.add("Rear Right Output", m_backRightOutput);

    m_speedsLayout =
        m_driveBaseTab.getLayout("Speeds", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 5);
    m_speedsLayout.addDouble("Front Left Speed", () -> getFrontLeftMetersPerSecond());
    m_speedsLayout.addDouble("Front Right Speed", () -> getFrontRightMetersPerSecond());
    m_speedsLayout.addDouble("Rear Left Speed", () -> getRearLeftMetersPerSecond());
    m_speedsLayout.addDouble("Rear Right Speed", () -> getRearRightMetersPerSecond());
    m_speedsLayout.addDouble("Rotation Rads/sec", () -> getRotationRadsPerSec());



  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_backLeftEncoder.getRate(),
        m_backRightEncoder.getRate());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_backLeftEncoder.getDistance(),
        m_backRightEncoder.getDistance());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    m_frontLeftOutput = m_frontLeftPIDController.calculate(m_frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    m_frontRightOutput = m_frontRightPIDController.calculate(m_frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    m_backLeftOutput = m_backLeftPIDController.calculate(m_backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    m_backRightOutput = m_backRightPIDController.calculate(m_backRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    m_frontLeftMotor.setVoltage(m_frontLeftOutput + frontLeftFeedforward);
    m_frontRightMotor.setVoltage(m_frontRightOutput + frontRightFeedforward);
    m_backLeftMotor.setVoltage(m_backLeftOutput + backLeftFeedforward);
    m_backRightMotor.setVoltage(m_backRightOutput + backRightFeedforward);

    m_speeds.frontLeftMetersPerSecond = speeds.frontLeftMetersPerSecond;
    m_speeds.frontRightMetersPerSecond = speeds.frontRightMetersPerSecond;
    m_speeds.rearLeftMetersPerSecond = speeds.rearLeftMetersPerSecond;
    m_speeds.rearRightMetersPerSecond = speeds.rearRightMetersPerSecond;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                                   : new ChassisSpeeds(xSpeed, ySpeed, rot),
                                     periodSeconds));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
    m_rotationRadsPerSec = rot;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getCurrentDistances());
  }

  public double getFrontLeftMetersPerSecond () { return m_speeds.frontLeftMetersPerSecond;}
  public double getFrontRightMetersPerSecond () { return m_speeds.frontRightMetersPerSecond;}
  public double getRearLeftMetersPerSecond () { return m_speeds.rearLeftMetersPerSecond;}
  public double getRearRightMetersPerSecond () { return m_speeds.rearRightMetersPerSecond;}
  public double getRotationRadsPerSec () { return m_rotationRadsPerSec;}
}
