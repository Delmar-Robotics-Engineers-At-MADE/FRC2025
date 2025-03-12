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

  public static final class ElPosition {
    public static final double MoveOffStart = 1500;
  }
  static final int CANIDPort = 20;
  static final int CANIDStar = 21;
  static final int DIONumPort = 4;
  static final int DIONumStar = 5;
  static final int HomeAngle = 0;
  static final double kFF = 2.2; // constant FF, not multiplied by gravity angle
  static final double PositionTolerance = 3; // degrees
  static final double VelocityV = 80000;  // degrees per minute
  static final double MRTOORTD = 360 / 3; // Motor Rotations To One Output Rotation To Degrees; main swerve is 5.49

  private SparkMax m_motorPort, m_motorStar;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopPort, closedLoopStar;
  private RelativeEncoder m_encoderPort, m_encoderStar;
  private DigitalInput m_magSwitchPort, m_magSwitchStar;
  private boolean m_homedPort = false, m_homedStar = false;
  private double m_holdPosPort = 0, m_holdPosStar = 0;

  // shuffleboard stuff
  private ShuffleboardTab m_matchTab = Shuffleboard.getTab("Match");
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Elevator");

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
        .p(1.5 /MRTOORTD) // 1.5
        .i(0.0001)
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
        .maxAcceleration(500*MRTOORTD, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000*MRTOORTD, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(MRTOORTD, ClosedLoopSlot.kSlot1); // degrees per sec

    motorConfig.idleMode(IdleMode.kBrake);

    m_motorPort.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // second motor inverted and following first
    motorConfig.inverted(true);
    m_motorStar.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Already homed?  Hope so, at beginning of match
    checkForHomePosition();

    // Dashboard indicators
    m_matchTab.addBoolean("El Port Homed", () -> getHomedPort());
    m_matchTab.addBoolean("El Star Homed", () -> getHomedStar());
    debugTab.addDouble("Port Angle", () -> getAnglePort());
    debugTab.addDouble("Star Angle", () -> getAngleStar());

    setDefaultCommand(new HoldElevatorCmd(this));
  
  }

  public void holdCurrentPosition (boolean port, boolean starboard) {
    System.out.println("holding current position");
    if (port) {
      m_holdPosPort = m_encoderPort.getPosition();
      closedLoopPort.setReference(m_holdPosPort, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF);
    }
    if (starboard) {
      double m_holdPosStar = m_encoderStar.getPosition();
      closedLoopStar.setReference(m_holdPosStar, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF);
    }
  }

  public void checkForHomePosition () {
    System.out.println("checking home of elevator");
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
      System.out.println("Moving to angle " + angle);
      closedLoopPort.setReference(angle, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0, kFF);
      closedLoopStar.setReference(angle, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0, kFF);
    } else {
      System.out.println("************* Both elevator motors not homed, can't move to position **********");
    }
  }

  public void moveVelocity (boolean up, boolean port, boolean starboard) {

    if (port && starboard) {System.out.println("moving both elevator motors");}
    else {System.out.println("moving elevator motor(s) " + up);}

    if (port) {
      if (!m_homedPort) {  // ok to move manually
        closedLoopPort.setReference(VelocityV * (up ? 1:-1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1, kFF);
      } else {
        System.out.println("************* Port elevator homed, can't move manually **********");
        closedLoopPort.setReference(m_holdPosPort, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      }
    } 

    if (starboard) {
      if (!m_homedStar) {  // ok to move manually
        closedLoopStar.setReference(VelocityV * (up ? 1:-1), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1, kFF);
      } else {
        System.out.println("************* Starboard elevator homed, can't move manually **********");
        closedLoopPort.setReference(m_holdPosStar, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF);
      }
    } 

    checkForHomePosition();
  }

  public Command moveVelocityCommand(boolean up, boolean port, boolean starboard) {
    return new RunCommand(() -> moveVelocity(up, port, starboard), this);
  }

  public Command moveToPositionCommand(double angle) {
    return new RunCommand(() -> moveToPosition(angle), this);
  }

  int ReefLevelAngle[] = {0, 450, 450, 600, 600};
  public Command moveToReefLevelCmd(int level) {
    return new RunCommand(() -> moveToPosition(ReefLevelAngle[level]), this);
  }

  public boolean getHomedPort () {return m_homedPort;}
  public boolean getHomedStar () {return m_homedStar;}
  public double getAnglePort () {return m_encoderPort.getPosition();}
  public double getAngleStar () {return m_encoderStar.getPosition();}

}
