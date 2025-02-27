// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Robot extends TimedRobot {
  private GenericEntry m_maxSpeed;
  // private SimpleWidget m_maxSpeedWidget;
  private DoubleSubscriber m_dblSub;

  @Override
  public void robotInit() {

    m_maxSpeed =
        Shuffleboard.getTab("Configuration")
            .add("Max Speed", 1)
            .withWidget("Number Slider")
            .withPosition(1, 1)
            .withSize(2, 1)
            .getEntry();

    // m_maxSpeedWidget =
    //     Shuffleboard.getTab("Configuration")
    //             .add("Max Speed 2", 1)
    //             .withWidget(BuiltInWidgets.kNumberSlider)
    //             .withPosition(1, 2)
    //             .withSize(2, 1);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Shuffleboard");
    m_dblSub = table.getDoubleTopic("Configuration/Max Speed").subscribe(0.0);
  }

  @Override
  public void teleopInit() {
    System.out.println(m_maxSpeed.getDouble(1.0));

    // try directly with network tables, because shuffleboard wasn't working...
    //  it works again after 2025.3.1 upgrade.
    System.out.println(m_dblSub.get());
  }
}
