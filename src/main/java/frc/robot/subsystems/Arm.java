// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {

    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;

    public Arm(){
        leftArmMotor = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);

        rightArmMotor.follow(leftArmMotor, true);

        configureMotors(leftArmMotor);
        configureMotors(rightArmMotor);
    }

    public void configureMotors(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
        //motor.burnFlash();
        //motor.clearFaults();
    }

    // Setting soft limit
    public void armSpeed(double speed) {
        if(isInBound(speed)){
            leftArmMotor.set(speed);
        }
        else {
            leftArmMotor.set(0);
            // System.out.println("Limits working-");
        }
        
    }

    public boolean isInBound(double speed){
        if((getPosition() < 3) && (speed < 0)) return false;
        if ((getPosition() > 113) && (speed > 0)) return false;
        return true;
    }

    public double getPosition(){
        return ((leftArmMotor.getEncoder().getPosition()) * 360.0) / 16.0;
    }

    public double getVelocity(){
        return leftArmMotor.getEncoder().getVelocity()  * 2 * Math.PI;
    }
    
    // public double getXAxisPosition() {
    //     return ArmConstants.To360Scope(getPosition());
    // }
    // public DoubleSupplier ffCalc() {
    //     return () -> -0.1 * Math.cos(Math.toRadians(getXAxisPosition()));
    // }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Encoder Value", getPosition());
        // SmartDashboard.putNumber("Offset Encoder Value", ArmConstants.To360Scope(getPosition()));
        SmartDashboard.putNumber("Arm Speeed", rightArmMotor.get());

    }


   
}
