// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import 
com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon extends SubsystemBase {
  /** Creates a new Pigeon. */
  private Pigeon2 pigeon;

  private double offsetPitch;

  public Pigeon() {
    pigeon = new Pigeon2(41);
    offsetPitch = pigeon.getPitch();
  }

  private Pigeon2 getPigeon() {
    return pigeon;
  }

  public double getPitch() {
    return pigeon.getPitch() - offsetPitch;
  }

  public double getYaw() {
    return pigeon.getYaw();
  }

  public double getRoll() {
    return pigeon.getRoll();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("Pitch", getPitch());
  }
}
