// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */

  private CANSparkMax frontLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backLeft;
  private CANSparkMax backRight;
  private DifferentialDrive drive;

  public Drivebase() {
  frontLeft = new CANSparkMax(3, MotorType.kBrushless);
  frontRight = new CANSparkMax(1, MotorType.kBrushless);
  backLeft = new CANSparkMax(4, MotorType.kBrushless);
  backRight = new CANSparkMax(2, MotorType.kBrushless);

  //configureMotors(frontLeft);
  //configureMotors(frontRight);
  //configureMotors(backLeft);
  //configureMotors(backRight);

  frontLeft.setInverted(false);
  backLeft.setInverted(false);
  
  backLeft.follow(frontLeft);
  backRight.follow(frontRight);

  drive = new DifferentialDrive(frontLeft, frontRight);
  }

  public void configureMotors(CANSparkMax motor){
    motor.clearFaults();
    motor.setSmartCurrentLimit(40);
    motor.burnFlash();
  }

  public void arcadeDrive (double speed, double rotation) {
    
    if(Math.abs(speed) < 0.1)
    {
      speed = 0.0;
    }
    if(Math.abs(rotation) < 0.1)
    {
      rotation = 0.0;
    }
    

    drive.arcadeDrive(rotation, speed);
    drive.feed();
  }
  

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("TOP LEFT VELOCITY", frontLeft.getEncoder().getVelocity());
    //This method will be called once per scheduler run
  }
}
