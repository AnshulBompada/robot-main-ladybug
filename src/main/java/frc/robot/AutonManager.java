package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.HoldArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AutonConstants;

public class AutonManager {

    public Drivebase robotDrive;
    public Arm robotArm;
    public Intake robotIntake;
    
    public AutonManager(Drivebase robotDrive, Arm robotArm, Intake robotIntake) {
        this.robotDrive = robotDrive;
        this.robotArm = robotArm;
        this.robotIntake = robotIntake;
    }

    public Command autonomousCmd(int auton) {

        switch(auton) {
          // ------ CABLE-SIDE AUTON ------
            case 1:
              return new SequentialCommandGroup (
                // ARM PLACE 
                armToPosition(AutonConstants.AUTON_SCORE_POS),
                cubeOut(),
                new WaitCommand(AutonConstants.AUTON_OUT_TAKE_WAIT),
                armToPosition(AutonConstants.AUTON_IDLE_POS),

                // DRIVE BACK
                driveBack(1.3)
              );

            // ------ MID AUTON ------
            case 2:
              return new SequentialCommandGroup (
                // ARM PLACE 
                armToPosition(AutonConstants.AUTON_SCORE_POS),
                cubeOut(),
                new WaitCommand(AutonConstants.AUTON_OUT_TAKE_WAIT),
                armToPosition(AutonConstants.AUTON_IDLE_POS),

                // DRIVE BACK
                driveBack(1.3)
              );

            // ------ NON-CABLE SIDE AUTON ------
            case 3:
            return new SequentialCommandGroup (
              // ARM PLACE 
              armToPosition(Constants.ArmConstants.IDLE_POS),
              cubeOut(),
              new WaitCommand(AutonConstants.AUTON_OUT_TAKE_WAIT),
              armToPosition(AutonConstants.AUTON_IDLE_POS),

              // DRIVE BACK
              driveBack(3.0)
            );

            // ------ JUST SHOOT ------
            case 4:
            return new SequentialCommandGroup (
              // ARM PLACE 
              armToPosition(AutonConstants.AUTON_SCORE_POS),
              cubeOut(),
              new WaitCommand(AutonConstants.AUTON_OUT_TAKE_WAIT),
              armToPosition(AutonConstants.AUTON_IDLE_POS),

              // DRIVE BACK
              driveBack(1.3)
            );

            default: 
            return new SequentialCommandGroup (
                new InstantCommand( () -> System.out.println("ERROR: Autonomous Failure."))
            );
        }
    }



    private Command driveBack(double time){
      return new DefaultDrive(() -> AutonConstants.AUTON_ARM_SPEED_BWD, () -> 0.0, robotDrive).withTimeout(time);
  }
  //not used until 2-pc/1&1/2pc auton is created
  private Command driveFront(double time){
      return new DefaultDrive(() -> AutonConstants.AUTON_ARM_SPEED_FWD, () -> 0.0, robotDrive).withTimeout(time);
  }

  private Command armToPosition(double pos){
    return new HoldArmCommand(robotArm, 
    () -> pos).withTimeout(1);
  }

  private Command cubeOut(){
    return new InstantCommand(()-> robotIntake.cubeOut());
  }

  // not used until 2-pc/1&1/2pc auton is created
  private Command cubeInConeOut(){
    return new InstantCommand(()-> robotIntake.cubeIn());
  }
}