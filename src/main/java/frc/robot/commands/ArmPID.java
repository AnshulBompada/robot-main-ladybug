
package frc.robot.commands;

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars;
import frc.robot.subsystems.Arm;

public class ArmPID extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Arm robotArm;
    private ProfiledPIDController pid;
//    private ArmFeedforward FF;
    private double setpoint;

    private double kP;
    private double kI; 
    private double kD;

    // private double ks;
    // private double kg;
    // private double kv;
    // private double ka;

    private double armVelocity; 
    private double armAcceleration;

    public ArmPID(Arm robotArm, double setpoint) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        GlobalVars.shouldHoldArm = false;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.018;
      kI = 0;
      kD = 0;

      // ks = ArmConstants.kS;
      // kg = ArmConstants.kG;
      // kv = ArmConstants.kV;
      // ka = ArmConstants.kA;

      armVelocity = 500;
      armAcceleration = 200;

      pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(armVelocity, armAcceleration));
      // FF = new ArmFeedforward(ks, kg, kv, ka);

      pid.setTolerance(2);
      // pid.enableContinuousInput(0, 360);

      pid.reset(robotArm.getPosition());

      System.out.println("Command TELEOP ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
      double pidCalc = pid.calculate(robotArm.getPosition(), setpoint);

      // SmartDashboard.putNumber("Output", pidCalc.getAsDouble());

      robotArm.armSpeed(pidCalc);
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.armSpeed(0);
      System.out.println("Command TELEOP ARM ALIGN has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}