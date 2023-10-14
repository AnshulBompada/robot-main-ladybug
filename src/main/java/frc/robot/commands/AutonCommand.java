// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Pigeon;

public class AutonCommand extends CommandBase {
  /** Creates a new AutonCommand. */
  private Pigeon gyro;
  private Drivebase drive;
  private double direction;

  public AutonCommand(Drivebase drive, Pigeon gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gyro = gyro;
    this.drive = drive;
    direction = 1.0;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Direction", direction);

    if(gyro.getPitch() > 0) direction = 1.0;
    else direction = -1.0;

    if(Math.abs(gyro.getPitch()) > AutonConstants.GYRO_TOLERANCE) drive.autoArcadeDrive(direction * AutonConstants.AUTON_DRIVE_SPEED, 0);
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
