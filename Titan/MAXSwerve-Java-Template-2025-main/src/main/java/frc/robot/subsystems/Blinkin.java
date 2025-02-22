// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LEDConstants;

public class Blinkin extends SubsystemBase {
  // private static Blinkin instance = null;
  // public Blinkin getInstance() {
  //   if (instance == null) {
  //     instance = new Blinkin();
  //   }
  //   else {
  //     instance = this;
  //   }
  //   return instance;
  // }
  private final Spark one;
  private final Spark two;
  private final Alliance m_allianceColor;
  /** Creates a new Blinkin. */
  public Blinkin() {
    one = new Spark(0);
    two = new Spark(1);
    m_allianceColor = DriverStation.getAlliance().get();
  }

  public void setColour(double input) {
    // System.out.println("Blinkin setting color to: " + input);
    one.set(input);
    two.set(input);
  }

  public void setAllianceColor() {
    
    if(m_allianceColor == Alliance.Red) {
      setColour(LEDConstants.red);
    } else {
      setColour(LEDConstants.blue);
    }
  }

  public Command setAllianceColorCmd() {
    return runOnce(() -> setAllianceColor());
  }

  public Command setCmd(double colour) {
    return runOnce(() -> setColour(colour));
  }

  public void setDefault() {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if(alliance.get() == Alliance.Red) {
        super.setDefaultCommand(redCmd());
      }
      else {
        super.setDefaultCommand(blueCmd());
      }
  }

  // public Command indCapture() {
  //   return new SequentialCommandGroup(
  //     set(LEDConstants.green),
  //     new WaitCommand(0.2),
  //     set(LEDConstants.grey),
  //     new WaitCommand(0.2),
  //     set(LEDConstants.green),
  //     new WaitCommand(0.2)
  //   );
  // }

  public Command purpleCmd() {
    return setCmd(LEDConstants.purple);
  }

  public Command greenCmd() {
    return setCmd(LEDConstants.green);
  }

  public Command redCmd() {
    return setCmd(LEDConstants.red);
  }

  public Command blueCmd() {
    return setCmd(LEDConstants.blue);
  }
}
