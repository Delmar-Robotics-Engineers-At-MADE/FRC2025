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
import frc.robot.commands.HoldElevatorCmd;

public class ElevatorSubsystem extends SubsystemBase{

  static final int CANIDPort = 1;
  static final int CANIDStar = 4;
  static final int DIONumPort = 0;
  static final int DIONumStar = 1;
  static final int HomeAngle = 0;
  static final double OpenLoopV = 10000;  // degrees per minute
  static final double MRTOORTD = 360 / 5.49; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopPort, closedLoopStar;
  private RelativeEncoder m_encoderPort, m_encoderStar;
  private DigitalInput m_magSwitchPort, m_magSwitchStar;
  private boolean m_homedPort = false, m_homedStar = false;

  // shuffleboard stuff
  private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  public ElevatorSubsystem() {
    m_magSwitchPort = new DigitalInput(DIONumPort);
    m_magSwitchStar = new DigitalInput(DIONumStar);
    m_motorPort = new SparkMax(CANIDPort, MotorType.kBrushless);
    m_motorStar = new SparkMax(CANIDStar, MotorType.kBrushless);
    closedLoopPort = m_motorPort.getClosedLoopController();
    closedLoopStar = m_motorStar.getClosedLoopController();
    m_encoderPort = m_motorPort.getEncoder();
    m_encoderStar = m_motorStar.getEncoder();

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
    motorConfig.inverted(true);
    m_motorStar.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Already homed?  Hope so, at beginning of match
    checkForHomePosition();

    // Dashboard indicators
    matchTab.addBoolean("El Port Homed", () -> getHomedPort());
    matchTab.addBoolean("El Star Homed", () -> getHomedStar());

    setDefaultCommand(new HoldElevatorCmd(this));
  
  }

  public void holdCurrentPosition (boolean port, boolean starboard) {
    System.out.println("holding current position");
    if (port) {
      double currPosPort = m_encoderPort.getPosition();
      closedLoopPort.setReference(currPosPort, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
    if (starboard) {
      double currPosStar = m_encoderStar.getPosition();
      closedLoopStar.setReference(currPosStar, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
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

  public void moveToPosition (double angle) {
    if (m_homedPort && m_homedStar) {
      closedLoopPort.setReference(angle, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0);
      closedLoopStar.setReference(angle, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0);
    } else {
      System.out.println("************* Both elevator motors not homed, can't move to position **********");
    }
  }

  public void moveOpenLoop (boolean port, boolean starboard) {

    if (port && starboard) {System.out.println("moving both elevator motors");}

    if (port) {
      if (!m_homedPort) {  // ok to move manually
        closedLoopPort.setReference(OpenLoopV, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
      } else {
        closedLoopPort.setReference(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
        System.out.println("************* Port elevator homed, can't move manually **********");
      }
    } else {  // not moving port, so hold it
      // holdCurrentPosition(true, false);
    }

    if (starboard) {
      if (!m_homedStar) {  // ok to move manually
        closedLoopStar.setReference(OpenLoopV, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
      } else {
        closedLoopStar.setReference(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
        System.out.println("************* Starboard elevator homed, can't move manually **********");
      }
    } else { // not moving starboard, so hold it
      // holdCurrentPosition(false, true);
    }
  }

  public Command moveOpenLoopCommand(boolean port, boolean starboard) {
    return new RunCommand(() -> moveOpenLoop(port, starboard), this);
  }

  public boolean getHomedPort () {return m_homedPort;}
  public boolean getHomedStar () {return m_homedStar;}

}
