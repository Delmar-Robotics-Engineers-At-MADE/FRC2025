package frc.robot.subsystems;

// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;
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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Hold2BarCmd;
import frc.robot.commands.HoldCoralCmd;
import frc.robot.commands.HoldElevatorCmd;

public class CoralSubsystem extends SubsystemBase{

  static final int CANIDMotor = 26;
  // static final int CANIDFusion = 1; // fusion line of flight sensor
  static final int DIONumPhotoEye = 2;
  static final double PositionTolerance = 10; // degrees
  static final double VelocityV = 15000;  // degrees per minute
  static final double MRTOORTD = 360 / 20; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoder;
  private boolean m_coralPresent = false;
  private double m_holdPosition = 0;
  private DigitalInput m_photoEye;
  // private final TimeOfFlight m_tofSensor;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Coral");

  public CoralSubsystem() {
    m_photoEye = new DigitalInput(DIONumPhotoEye);
    // m_tofSensor = new TimeOfFlight(CANIDFusion);
    // m_tofSensor.setRangingMode(RangingMode.Short, 200); // msecs

    m_motor = new SparkMax(CANIDMotor, MotorType.kBrushless);
    closedLoopController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

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
        .allowedClosedLoopError(PositionTolerance) // in degrees
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(1000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .maxVelocity(60000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(MRTOORTD, ClosedLoopSlot.kSlot1); // degrees per sec

    motorConfig.idleMode(IdleMode.kBrake);

    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Dashboard indicators
    matchTab.addBoolean("Coral Present", () -> getCoralPresent())
        .withPosition(1, 4);
    debugTab.addDouble("Position", () -> getAngle());

    setDefaultCommand(new HoldCoralCmd(this));
  
  }

  public void holdCurrentPosition () {
    m_holdPosition = m_encoder.getPosition();
    System.out.println("coral holding current position: " + m_holdPosition);
    closedLoopController.setReference(m_holdPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public void moveVelocityControl (boolean in) {
    closedLoopController.setReference(VelocityV * (in?-1:1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
  }
  public void stop () {
    closedLoopController.setReference(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
  }

  public Command moveVelocityOnceCmd(boolean in) {
    System.out.println("coral moving " + in);
    return new InstantCommand(() -> moveVelocityControl(in), this);
  }
  public Command moveVelocityCmd(boolean in) {
    System.out.println("coral moving " + in);
    return new RunCommand(() -> moveVelocityControl(in), this);
  }
  public Command stopCommand() {
    return new InstantCommand(() -> stop(), this);
  }

  public boolean getCoralPresent () {return m_photoEye.get() == false;}
  public double getAngle () {return m_encoder.getPosition();}

}
