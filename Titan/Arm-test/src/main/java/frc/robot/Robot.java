/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  public Robot() {
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor = new SparkMax(15, MotorType.kBrushless);
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
        .positionConversionFactor(360/4) // everything is degrees of arm rotation
        .velocityConversionFactor(360/4);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.005)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 57.67, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000000)
        .maxAcceleration(1000000)
        .allowedClosedLoopError(10) 
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(1000000, ClosedLoopSlot.kSlot1)
        .maxVelocity(1000000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(10, ClosedLoopSlot.kSlot1);

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

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("kFF", 0);
    // SmartDashboard.setDefaultNumber("Target Velocity", 0);
    // SmartDashboard.setDefaultBoolean("Control Mode V", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  @Override
  public void teleopPeriodic() {
    // if (SmartDashboard.getBoolean("Control Mode V", false)) {
      /*
       * Get the target velocity from SmartDashboard and set it as the setpoint
       * for the closed loop controller with MAXMotionVelocityControl as the
       * control type.
       */
    //   double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
    //   closedLoopController.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl,
    //       ClosedLoopSlot.kSlot1);
    // } else {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller with MAXMotionPositionControl as the
       * control type.
       */
      double targetPosition = SmartDashboard.getNumber("Target Position", 0);
      SmartDashboard.putNumber("Target Position 2", targetPosition);
      double kFF = SmartDashboard.getNumber("kFF", 0);
      double angle = Math.toRadians(encoder.getPosition());  // calculate angle in rads
      double feedForward = kFF * Math.sin(angle);
      closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0, feedForward);
    // }
  }

  @Override
  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putNumber("Actual Position", encoder.getPosition());
    // SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      encoder.setPosition(0);
    }
  }
}
