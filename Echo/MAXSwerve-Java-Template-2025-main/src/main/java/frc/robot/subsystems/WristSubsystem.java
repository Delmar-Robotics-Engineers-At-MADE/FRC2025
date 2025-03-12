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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HoldWristCmd;

public class WristSubsystem extends SubsystemBase{

  public static final class WristPosition {
    public static final double CoralStation = 95;
  }

  static final int CANIDPort = 24;
  static final int CANIDStar = 25;
  static final int DIONum = 3;
  static final int HomeAngle = 0;
  static final double kFF = 2.5;
  static final double PositionTolerance = 2; // degrees
  static final double VelocityV = 10000;  // degrees per minute
  static final double MRTOORTD = 360 / 6; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoderPort;
  private DigitalInput m_photoEye;
  private boolean m_homed = false;
  private double m_holdPosition = 0;
  private boolean m_overtempPort, m_overtempStar;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Wrist");

  public WristSubsystem() {
    m_photoEye = new DigitalInput(DIONum);
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
        .d(0.11)
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
        .allowedClosedLoopError(PositionTolerance) // in degrees
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500*MRTOORTD, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(MRTOORTD, ClosedLoopSlot.kSlot1); // degrees per sec

    motorConfig.idleMode(IdleMode.kBrake);

    m_motorPort.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // second motor inverted and following first
    motorConfig.follow(CANIDPort, true);
    m_motorStar.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Already homed?  Hope so, at beginning of match
    checkForHomePosition();

    // Dashboard indicators
    matchTab.addBoolean("Wrist Homed", () -> getHomed()).withPosition(7, 0);
    matchTab.addBoolean("Wrist Hot Port", () -> getTempGoodPort()).withPosition(8, 0);
    matchTab.addBoolean("Wrist Hot Star", () -> getTempGoodStar()).withPosition(9, 0);
    debugTab.addDouble("Angle", () -> getAngle());

    // setDefaultCommand(new HoldWristCmd(this));
  
  }

  public void holdCurrentPosition () {
    m_holdPosition = m_encoderPort.getPosition();
    System.out.println("Wrist holding current position " + m_holdPosition);
    double currAngle = Math.toRadians(m_encoderPort.getPosition());  // calculate angle in rads
    double feedForward = kFF * Math.sin(currAngle);
    closedLoopController.setReference(m_holdPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedForward);
  }

  public void checkForHomePosition () {
    System.out.println("checking home of wrist");
    // if (!m_homed && m_photoEye.get() == false) { // false means pressed
    if (!m_homed) {
      m_homed = true;
      m_encoderPort.setPosition(HomeAngle);
    }
  }

  static final double MaxTemp = 100; // celsius
  public void moveToPosition (double angle) {
    if (m_homed) {
      m_overtempPort = m_motorPort.getMotorTemperature() < MaxTemp;
      m_overtempStar = m_motorStar.getMotorTemperature() < MaxTemp;
      if (m_overtempPort || m_overtempStar) {
        // motor(s) too hot, move to home position instead
        angle = 0;
        System.out.println("************* Wrist motors too hot; homing **********");
      }
      double currAngle = Math.toRadians(m_encoderPort.getPosition());  // calculate angle in rads
      double feedForward = kFF * Math.sin(currAngle);
      closedLoopController.setReference(angle, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0, feedForward);
    } else {
      System.out.println("************* Wrist motor not homed, can't move to position **********");
    }
  }

  public void moveVelocity (boolean up) {
    System.out.println("Wrist moving open loop");
    if (!m_homed) {  // ok to move manually
      closedLoopController.setReference(VelocityV * (up?1:-1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    } else {
      double currAngle = Math.toRadians(m_encoderPort.getPosition());  // calculate angle in rads
      double feedForward = kFF * Math.sin(currAngle);
      closedLoopController.setReference(m_holdPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedForward);
      System.out.println("************* Wrist homed, can't move manually **********");
    }

    checkForHomePosition();
  }

  public Command moveVelocityCommand(boolean up) {
    return new RunCommand(() -> moveVelocity(up), this);
  }

  double ReefLevelAngle[] = {0, 45, 45, 60, 60};
  public Command moveToReefLevel(int level) {
    return new RunCommand(() -> moveToPosition(ReefLevelAngle[level]), this);
  }

  static final double CoralStationAngle = 20;
  public Command moveToCoralStation() {
    return new RunCommand(() -> moveToPosition(CoralStationAngle), this);
  }

  public boolean getHomed () {return m_homed;}
  public boolean getTempGoodPort () {return !m_overtempPort;}
  public boolean getTempGoodStar () {return !m_overtempStar;}
  public boolean getPhotoEye () {return m_photoEye.get();}
  public double getAngle () {return m_encoderPort.getPosition();}

}
