// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {

    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;

    public double lastSetpoint;

    public Arm(){
        leftArmMotor = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);

        rightArmMotor.follow(leftArmMotor, true);

        configureMotors(leftArmMotor);
        configureMotors(rightArmMotor);

        lastSetpoint = getPosition().getDegrees();
    }

    public void configureMotors(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
        //motor.burnFlash();
        //motor.clearFaults();
    }

    // Setting soft limit
    public void armSpeedVolt(double volts) {
        if (isInBound(volts)) leftArmMotor.setVoltage(volts);
        else leftArmMotor.setVoltage(0);

        // System.out.println("Limits working-");
    }

    public boolean isInBound(double volts){
        if((getPosition().getDegrees() < ArmConstants.LOWER_BOUND) && (volts < 0)) return false;
        if ((getPosition().getDegrees() > ArmConstants.UPPER_BOUND) && (volts > 0)) return false;
        return true;
    }

    public Rotation2d getPosition(){
        return new Rotation2d(Math.toRadians((leftArmMotor.getEncoder().getPosition() * 360.0) / ArmConstants.ARM_GEAR_RATIO));
    }

    public Rotation2d getVelocity(){
        return new Rotation2d(Units.rotationsPerMinuteToRadiansPerSecond(leftArmMotor.getEncoder().getVelocity()));
    }
    
    public double getLastSetpoint() {
        return lastSetpoint;
    }

    public void setLastSetpoint(double setpoint) {
        lastSetpoint = setpoint;
    }

    // public double getXAxisPosition() {
    //     return ArmConstants.To360Scope(getPosition());
    // }
    // public DoubleSupplier ffCalc() {
    //     return () -> -0.1 * Math.cos(Math.toRadians(getXAxisPosition()));
    // }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Encoder Value", getPosition().getDegrees());
        // SmartDashboard.putNumber("Offset Encoder Value", ArmConstants.To360Scope(getPosition()));
        SmartDashboard.putNumber("Arm Speeed", rightArmMotor.get());
        SmartDashboard.putNumber("Right Arm Current", rightArmMotor.getOutputCurrent());
        SmartDashboard.putNumber("Left Arm Current", leftArmMotor.getOutputCurrent());

    }


   
}
