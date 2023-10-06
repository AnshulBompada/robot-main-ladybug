// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.NullSerializer;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Pigeon;

public class AutonCommand extends ProfiledPIDCommand {
  /* Creates a new AutonCommand. */
  
  private Drivebase drive;
  private Pigeon pigeon;

  public AutonCommand(Drivebase drive, Pigeon pigeon) {
    super(
      new ProfiledPIDController(
        0.01, 
        0, 
        0, 
        new TrapezoidProfile.Constraints(1.5, 1)),
        () -> pigeon.getPitch(), 
        AutonConstants.GYRO_GOAL_POS, 
        (output, setpoint) -> { 
          SmartDashboard.getNumber("output", output);
          System.out.println(output);
          drive.autoArcadeDrive(output, 0);}, 
        drive
      );

    addRequirements(drive);
    getController().setTolerance(2.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
