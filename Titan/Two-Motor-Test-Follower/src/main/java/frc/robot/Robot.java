/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


public class Robot extends TimedRobot {

  static final int MotorCANID = 4;
  static final double MRTOORTD = 360 / 5.49; // Motor Rotations To One Output Rotation To Degrees

  private SparkMax motor, motor2;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  private ShuffleboardTab shuffTab = Shuffleboard.getTab("Motor");
  private GenericEntry shuffTargetV = shuffTab.add("Target Velocity (degrees per min)", 0).getEntry();
  private GenericEntry shuffTargetP = shuffTab.add("Target Position (degrees)", 0).getEntry();
  private GenericEntry shuffKFF = shuffTab.add("kFF", 0).getEntry();
  private GenericEntry shuffVControl = shuffTab
      .add("Velocity Ctrl", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();
  private GenericEntry shuffResetEncoder = shuffTab
      .add("Reset Encoder", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

  public Robot() {
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor = new SparkMax(4, MotorType.kBrushless);
    motor2 = new SparkMax(1, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(MRTOORTD)
        .velocityConversionFactor(MRTOORTD);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4 /MRTOORTD)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001/MRTOORTD, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / (5767*MRTOORTD), ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000*MRTOORTD)
        .maxAcceleration(1000*MRTOORTD)
        .allowedClosedLoopError(20) // in degrees
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500*MRTOORTD, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(MRTOORTD, ClosedLoopSlot.kSlot1); // degrees per sec

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // second motor inverted and following first
    motorConfig.follow(4, true);
    motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // set up read-only widgets of dashboard
    shuffTab.addDouble("Actual Position", () -> encoder.getPosition());
    shuffTab.addDouble("Actual Velocity", () -> encoder.getVelocity());

  }

  @Override
  public void teleopPeriodic() {
    if (shuffVControl.getBoolean(false)) {
      /*
       * Get the target velocity from Shuffleboard and set it as the setpoint
       * for the closed loop controller with MAXMotionVelocityControl as the
       * control type.
       */
      double targetVelocity = shuffTargetV.getDouble(0);
      closedLoopController.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot1);
    } else {
      /*
       * Get the target position from Shuffleboard and set it as the setpoint
       * for the closed loop controller with MAXMotionPositionControl as the
       * control type.
       */
      double targetPosition = shuffTargetP.getDouble(0);
      double kFF = shuffKFF.getDouble(0);
      double angle = Math.toRadians(encoder.getPosition());  // calculate angle in rads
      double feedForward = kFF * Math.sin(angle);
      closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0, feedForward);
    }
  }

  @Override
  public void robotPeriodic() {
    // System.out.println("Button: " + shuffResetEncoder.getBoolean(false));

    if (shuffResetEncoder.getBoolean(false)) {
      System.out.println("Resetting encoder");
      shuffResetEncoder.setBoolean(false);
      // Reset the encoder position to 0
      encoder.setPosition(0);
    }
  }
}
