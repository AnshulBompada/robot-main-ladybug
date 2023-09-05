package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmPID;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;

public class AutonManager {

    public Drivebase robotDrive;
    public Arm robotArm;
    public Intake robotIntake;
    private double armSpeedFwd;
    private double armSpeedBwd;
    
    public AutonManager(Drivebase robotDrive, Arm robotArm, Intake robotIntake) {
        this.robotDrive = robotDrive;
        this.robotArm = robotArm;
        this.robotIntake = robotIntake;
    }

    public Command autonomousCmd(int auton) {
        armSpeedFwd = 0.5;
        armSpeedBwd = -0.3;

        double outTakeWait = 1;
        double scorePos = 45;

        switch(auton) {
          // ------ CABLE-SIDE AUTON ------
            case 1:
              return new SequentialCommandGroup (
                // ARM PLACE 
                armToPosition(scorePos),
                coneInCubeOut(),
                new WaitCommand(outTakeWait),
                armToPosition(0),

                // DRIVE BACK
                driveBack(1.3)
              );

            // ------ MID AUTON ------
            case 2:
              return new SequentialCommandGroup (
                // ARM PLACE 
                armToPosition(scorePos),
                coneInCubeOut(),
                new WaitCommand(outTakeWait),
                armToPosition(0),

                // DRIVE BACK
                driveBack(1.3)
              );

            // ------ NON-CABLE SIDE AUTON ------
            case 3:
            return new SequentialCommandGroup (
              // ARM PLACE 
              armToPosition(scorePos),
              coneInCubeOut(),
              new WaitCommand(outTakeWait),
              armToPosition(0),

              // DRIVE BACK
              driveBack(1.3)
            );

            // ------ JUST SHOOT ------
            case 4:
            return new SequentialCommandGroup (
              // ARM PLACE 
              armToPosition(scorePos),
              coneInCubeOut(),
              new WaitCommand(outTakeWait),
              armToPosition(0),

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
      return new DefaultDrive(() -> armSpeedBwd, () -> 0.0, robotDrive).withTimeout(time);
  }

  private Command driveFront(double time){
      return new DefaultDrive(() -> armSpeedFwd, () -> 0.0, robotDrive).withTimeout(time);
  }

  private Command armToPosition(double pos){
    return new ArmPID(robotArm, pos).withTimeout(1);
  }

  private Command coneInCubeOut(){
    return new InstantCommand(()-> robotIntake.coneIn());
  }

  private Command cubeInConeOut(){
    return new InstantCommand(()-> robotIntake.cubeIn());
  }
}