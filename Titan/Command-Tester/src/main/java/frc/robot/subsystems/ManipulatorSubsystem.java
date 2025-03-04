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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Hold2BarCmd;
import frc.robot.commands.HoldElevatorCmd;

public class ManipulatorSubsystem extends SubsystemBase{

  static final int CANIDPort = 2;
  static final int CANIDStar = 3;
  static final int DIONumPort = 2;
  static final int DIONumStar = 3;
  static final int DIONumCoral = 4;
  static final int HomeAngle = 0;
  static final double OpenLoopV = 10000;  // degrees per minute
  static final double MRTOORTD = 360 / 5.49; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder m_encoderPort, m_encoderStar;
  private DigitalInput m_magSwitchPort, m_magSwitchStar, m_photoCoral;
  private boolean m_homedPort = false, m_homedStar = false, m_coralPresent = false;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  public ManipulatorSubsystem() {
    m_magSwitchPort = new DigitalInput(DIONumPort);
    m_magSwitchStar = new DigitalInput(DIONumStar);
    m_photoCoral = new DigitalInput(DIONumCoral);
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

    // Already homed?  Hope so, at beginning of match
    checkForHomePosition();
    checkForCoral();

    // Dashboard indicators
    matchTab.addBoolean("2Bar Port Homed", () -> getHomedPort());
    matchTab.addBoolean("2Bar Star Homed", () -> getHomedStar());
    matchTab.addBoolean("Coral Present", () -> getCoralPresent());

    setDefaultCommand(new Hold2BarCmd(this));
  
  }

  public void holdCurrentPosition () {
    System.out.println("2 bar holding current position");
    double currPosPort = m_encoderPort.getPosition();
    closedLoopController.setReference(currPosPort, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  private void checkForHomePosition () {
    // System.out.println("checking home of elevator");
    if (!m_homedPort && m_magSwitchPort.get() == false) { // false means pressed
      m_homedPort = true;
      m_encoderPort.setPosition(HomeAngle);
    }
    if (!m_homedStar && m_magSwitchStar.get() == false) { // false means pressed
      m_homedStar = true;
      m_encoderStar.setPosition(HomeAngle);
    }
  }

  private void checkForCoral () {
    System.out.println("checking for coral");
    m_coralPresent = (m_photoCoral.get() == false); // false means present
  }

  public void moveToPosition (double angle) {
    if (m_homedPort && m_homedStar) {
      closedLoopController.setReference(angle, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0);
    } else {
      System.out.println("************* Both 2 bar motors not homed, can't move to position **********");
    }
  }

  public void moveOpenLoop (boolean up) {
    if (!m_homedPort || !m_homedStar) {  // ok to move manually
      closedLoopController.setReference(OpenLoopV * (up?1:-1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    } else {
      closedLoopController.setReference(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
      System.out.println("************* 2 bar homed, can't move manually **********");
    }

    checkForHomePosition();
  }

  public Command moveOpenLoopCommand(boolean up) {
    return new RunCommand(() -> moveOpenLoop(up), this);
  }

  public boolean getCoralPresent () {/*return m_coralPresent;*/ return m_photoCoral.get();}
  public boolean getHomedPort () {return m_homedPort;}
  public boolean getHomedStar () {return m_homedStar;}

}
