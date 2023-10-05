// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.HoldArmCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private Drivebase drive;
  private CommandXboxController driveController;
  private CommandXboxController operatorController;
  private Intake intake;
  private Arm arm;
  private AutonManager autonManager;
  private SendableChooser<Command> autonChooser;
  private HoldArmCommand holdArmCmd;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    intake = new Intake();
    arm = new Arm();
    drive = new Drivebase();
    
    autonManager = new AutonManager(drive, arm, intake);
    autonChooser = new SendableChooser<>();
    
    driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROL_PORT);
    operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROL_PORT);

    holdArmCmd = new HoldArmCommand(arm, () -> arm.getLastSetpoint() );

    drive.setDefaultCommand(new DefaultDrive(
      () -> driveController.getLeftY(),
      () -> driveController.getRightX(),
      drive
    ));

    arm.setDefaultCommand(holdArmCmd);
    
    Shuffleboard.getTab("Autonomous: ").add(autonChooser);
    autonChooser.addOption("SCORE MOBILITY, CABLE SIDE", autonManager.autonomousCmd(1));
    autonChooser.addOption("SCORE MOBILITY DOCK", autonManager.autonomousCmd(2));
    autonChooser.addOption("SCORE MOBILITY, NO CABLE", autonManager.autonomousCmd(3));
    autonChooser.setDefaultOption("SCORE", autonManager.autonomousCmd(4));
    
    configureBindings();
  }

  
  private void configureBindings() {
    operatorController.leftTrigger()
      .onTrue(new InstantCommand(() -> arm.setShouldPID(false)))
      .whileTrue(new InstantCommand(() -> arm.armSpeedVolt(ArmConstants.ARM_VOLT_FWD)))
      .onFalse(new InstantCommand(() -> {
          arm.setShouldPID(true); 
          arm.setLastSetpoint(arm.getPosition().getDegrees()); }));

    operatorController.rightTrigger()
      .onTrue(new InstantCommand(() -> arm.setShouldPID(false)))
      .whileTrue(new InstantCommand(() -> arm.armSpeedVolt(ArmConstants.ARM_VOLT_BWD)))
      .onFalse(new InstantCommand(() -> {
        arm.setShouldPID(true); 
        arm.setLastSetpoint(arm.getPosition().getDegrees()); }));

    operatorController.leftBumper()
      .onTrue(new InstantCommand(() -> {
        holdArmCmd.resetPID();
        arm.setLastSetpoint(Constants.ArmConstants.FLOOR_POS); }))
      .onFalse(new InstantCommand(() -> arm.setLastSetpoint(Constants.ArmConstants.IDLE_POS)));

   
    operatorController.rightBumper()
    .onTrue(new InstantCommand(() -> {
      holdArmCmd.resetPID();
      arm.setLastSetpoint(Constants.ArmConstants.FLOOR_POS); }))
    .onFalse(new InstantCommand(() -> arm.setLastSetpoint(Constants.ArmConstants.IDLE_POS)));

    operatorController.a().onTrue(new InstantCommand(() -> intake.cubeIn()));
    operatorController.b().onTrue(new InstantCommand(() -> intake.cubeOut()));

    operatorController.a().onFalse(new InstantCommand(() -> intake.setZero()));
    operatorController.b().onFalse(new InstantCommand(() -> intake.setZero()));
  }

  public Arm getRobotArm(){
    return arm;
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
