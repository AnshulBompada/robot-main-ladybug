// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CubeConstants;;



public class Intake extends SubsystemBase {
  
  public CANSparkMax intakeMotor;
  
  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT); // if stalling, set to 30 amps
  }

  public void configMotor(CANSparkMax motor){
    motor.restoreFactoryDefaults();
    motor.clearFaults();
  }

  public void cubeIn(){
    intakeMotor.set(CubeConstants.CUBE_IN_SPEED);
  }

  public void cubeOut(){
    intakeMotor.set(CubeConstants.CUBE_OUT_SPEED);
  }

  public void setZero(){
    intakeMotor.set(0.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Intake Voltage", intakeMotor.get());
  }
}
