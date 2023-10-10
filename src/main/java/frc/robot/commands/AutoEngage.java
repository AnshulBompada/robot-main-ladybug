// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Pigeon;

public class AutoEngage extends CommandBase {
  /** Creates a new AutoEngage. */
  ProfiledPIDController controller;
  Drivebase drive;
  Pigeon pigeon;
  double setpoint;

  public AutoEngage(Drivebase drive, Pigeon pigeon) {
      controller = new ProfiledPIDController(0.45, 0, 0, new Constraints(1.5, 1.0));
      setpoint = 0;
      controller.setTolerance(2);
      this.drive = drive;
      this.pigeon = pigeon;

      addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset(pigeon.getPitch());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tempOutput = - controller.calculate(pigeon.getPitch(), setpoint);
    SmartDashboard.putNumber("output", tempOutput);
    drive.arcadeDrive(tempOutput, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
