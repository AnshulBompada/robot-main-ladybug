// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmPID;
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


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    intake = new Intake();
    arm = new Arm();
    drive = new Drivebase();
    
    autonManager = new AutonManager(drive, arm, intake);
    autonChooser = new SendableChooser<>();
    
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    drive.setDefaultCommand(new DefaultDrive(
      () -> driveController.getLeftY(),
      () -> driveController.getRightX(),
      drive
    ));

    // arm.setDefaultCommand(new HoldArmCommand(arm));
    
    Shuffleboard.getTab("Autonomous: ").add(autonChooser);
    autonChooser.addOption("SCORE MOBILITY, CABLE SIDE", autonManager.autonomousCmd(1));
    autonChooser.addOption("SCORE MOBILITY DOCK", autonManager.autonomousCmd(2));
    autonChooser.addOption("SCORE MOBILITY, NO CABLE", autonManager.autonomousCmd(3));
    autonChooser.setDefaultOption("SCORE", autonManager.autonomousCmd(4));
    
    configureBindings();
  }

  
  private void configureBindings() {
    operatorController.rightTrigger()
      .whileTrue(new InstantCommand(() -> {
                                            arm.armSpeed(0.25);
                                            stopArmHold();
                                          }))
      .onFalse(new InstantCommand(() -> beginArmHold()));

    operatorController.leftTrigger()
      .whileTrue(new InstantCommand(() -> {
                                            arm.armSpeed(-0.4);
                                            stopArmHold();
                                          }))
      .onFalse(new InstantCommand(() -> beginArmHold()));

    operatorController.leftBumper()
      .whileTrue(new ArmPID(arm, 90.0))
      .onFalse(new InstantCommand(() -> beginArmHold()));
   
    operatorController.rightBumper()
      .whileTrue(new ArmPID(arm, 5.0))
      .onFalse(new InstantCommand(() -> beginArmHold()));

    operatorController.a().onTrue(new InstantCommand(() -> intake.cubeIn()));
    operatorController.b().onTrue(new InstantCommand(() -> intake.cubeOut()));

    operatorController.a().onFalse(new InstantCommand(() -> intake.setZero()));
    operatorController.b().onFalse(new InstantCommand(() -> intake.setZero()));

    // operatorController.a().onTrue(new ArmPID(arm, 3));
    // operatorController.b().onTrue(new ArmPID(arm, 90));
  }

  private void stopArmHold(){
    System.out.println("ARM HOLD STOPPED");
    GlobalVars.shouldHoldArm = false;
  }


  private void beginArmHold(){
    System.out.println("ARM HOLD START");
    arm.armSpeed(0.0);
    GlobalVars.shouldHoldArm = true;
  }

  public Arm getRobotArm(){
    return arm;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonChooser.getSelected();
  }
}
