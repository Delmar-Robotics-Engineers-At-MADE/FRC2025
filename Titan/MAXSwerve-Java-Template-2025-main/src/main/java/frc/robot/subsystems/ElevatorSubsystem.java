package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

  static final int CANIDPort = 20;
  static final int CANIDStar = 21;
  static final int DIONum = 0;
  static final int HomeAngle = 0;
  static final double MRTOORTD = 360 / 5.49; // Motor Rotations To One Output Rotation To Degrees

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoderPort;
  private DigitalInput m_magnetSwitch;
  private boolean m_homed = false;

  public ElevatorSubsystem() {
    m_magnetSwitch = new DigitalInput(DIONum);
    m_motorPort = new SparkMax(CANIDPort, MotorType.kBrushless);
    m_motorStar = new SparkMax(CANIDStar, MotorType.kBrushless);
    closedLoopController = m_motorPort.getClosedLoopController();
    m_encoderPort = m_motorPort.getEncoder();

    motorConfig = new SparkMaxConfig();
    motorConfig.encoder
        .positionConversionFactor(MRTOORTD)
        .velocityConversionFactor(MRTOORTD);

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
        .allowedClosedLoopError(5) // in degrees
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500*MRTOORTD, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(MRTOORTD, ClosedLoopSlot.kSlot1); // degrees per sec

    m_motorPort.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // second motor inverted and following first
    motorConfig.follow(CANIDPort, true);
    m_motorStar.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void holdCurrentPosition () {
    double currPos = m_encoderPort.getPosition();
    closedLoopController.setReference(currPos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public Command holdCurrentPositionCmd () {
    return runOnce(() -> holdCurrentPosition()); 
  }

  private void checkForHomePosition () {
    if (!m_homed && m_magnetSwitch.get()) {
      m_homed = true;
      m_encoderPort.setPosition(HomeAngle);
    }
  }
}
