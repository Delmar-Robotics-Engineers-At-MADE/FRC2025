// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Slew rate limit is no longer part of template, since the wheels are better now, was...
    // public static final double kDirectionSlewRate = 1.2; // radians per second
    // public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    // public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 11;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = true;

    // public static final double kYawP = 0.03;
    // public static final double kYawI = 0.0;
    // public static final double kYawD = 0.0;
    // public static final double kMaxYawRateDegPerS = 8;
    // public static final double kMaxYawAccelerationDegPerSSquared = 20;
    // public static final double kYawToleranceDeg = 5;
    // public static final double kYawRateToleranceDegPerS = 10;
    // public static final double kLongToleranceMeter = 0.1;
    // public static final double kLatToleranceMeter = 0.1;

    // // react to April Tag yaw
    // public static final double kYawAprilP = 0.3;
    // public static final double kYawAprilI = 0;
    // public static final double kYawAprilD = 0.05;  
    // public static final double kYawAprilZero = 0.0; // 1.2;
    
    // // react to Limelight yaw
    // public static final double kYawLimeP = 0.04;
    // public static final double kYawLimeI = 0;
    // public static final double kYawLimeD = 0.001;  
    // public static final double kTurnToleranceLime = .1;
    // public static final double kTurnRateToleranceLimesPerS = 1; // meters (X of photon view) per second
    
    // // react to April Tag skew
    // public static final double kSkewAprilP = 0.5;
    // public static final double kSkewAprilI = 0.008;
    // public static final double kSkewAprilD = 0.01;  
    // public static final double kSkewAprilZero = Math.PI;  

    // // distance to April Tag
    // public static final double kDriveAprilP = .8;
    // public static final double kDriveAprilI = 0.0;
    // public static final double kDriveAprilD = 0.1;
    // public static final double kDriveAprilToleranceDist = 0.2;  // in meters
    // public static final double kMaxMetersPerS = 100;  // we don't really need this to be profiled, so set high number
    // public static final double kMaxMetersPerSSquared = 100;
    // public static final double kDriveAprilTarget = 2;
    
    // turn rate limits, only meaningful when error measurement is angle
    // public static final double kMaxTurnRateDegPerS = 180;
    // public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    // public static final double kTurnToleranceDeg = 2;
    // public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    // public static final double kMaxTurnRateRadPerS = 2 * Math.PI;
    // public static final double kMaxTurnAccelerationRadPerSSquared = 2 * Math.PI;
    // public static final double kTurnToleranceRad = Math.PI/60;
    // public static final double kTurnRateToleranceRadPerS = Math.PI/2; // degrees per second

    // public static final double kMaxTurnRateMPerS = 2;
    // public static final double kMaxTurnAccelerationMPerSSquared = 4;
    // public static final double kTurnToleranceM = .1;
    // public static final double kTurnRateToleranceMPerS = 1; // meters (X of photon view) per second
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    // public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
    //     / kDrivingMotorReduction; // meters
    // public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
    //     / kDrivingMotorReduction) / 60.0; // meters per second

    // public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    // public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    // public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    // public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    // These are in Configs.java now, was...
    // public static final double kDrivingP = 0.04;
    // public static final double kDrivingI = 0;
    // public static final double kDrivingD = 0;
    // public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    // public static final double kDrivingMinOutput = -1;
    // public static final double kDrivingMaxOutput = 1;

    // public static final double kTurningP = 1;
    // public static final double kTurningI = 0;
    // public static final double kTurningD = 0;
    // public static final double kTurningFF = 0;
    // public static final double kTurningMinOutput = -1;
    // public static final double kTurningMaxOutput = 1;

    // public static final int kDrivingMotorCurrentLimit = 50; // amps
    // public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kOperatorControllerPort = 2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    public static final double kTolerance = 0.3; // position control, meters

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LEDConstants {
    public static final double green = 0.77;
    public static final double purple = 0.91;
    public static final double red = -0.31;
    public static final double blue = -0.29;
    public static final double grey = -0.33;
  }

}
