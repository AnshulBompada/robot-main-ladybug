
package frc.robot.commands;
import edu.wpi.first.math.controller.ArmFeedforward;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars;
//import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;

public class HoldArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Arm robotArm;

    private ArmFeedforward FF;
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    public HoldArmCommand(Arm robotArm) {
        this.robotArm = robotArm;
        addRequirements(robotArm);
    }

    @Override
    public void initialize() {
      ks = 0;
      kg = -0.075;
      kv = 0;
      ka = 0;

      FF = new ArmFeedforward(ks, kg, kv, ka);
      System.out.println("Command HOLD ARM COMMAND has started");
    }
  
    @Override
    public void execute() {
      if(GlobalVars.shouldHoldArm){
        double calc = FF.calculate(
          Math.toRadians(robotArm.getPosition() - ArmConstants.ARM_PARALLEL_TO_GROUND), // -91.0 = arm parallel to ground
          Math.toRadians(robotArm.getVelocity()) / 12);

          robotArm.armSpeed(calc);  
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.armSpeed(0);
      System.out.println("Command HOLD ARM COMMAND has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }