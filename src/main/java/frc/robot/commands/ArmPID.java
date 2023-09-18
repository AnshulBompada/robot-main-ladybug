
package frc.robot.commands;

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;

public class ArmPID extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Arm robotArm;
    private ProfiledPIDController pid;
//    private ArmFeedforward FF;
    private double setpoint;

    private double kP;
    private double kI; 
    private double kD;

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

      pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(ArmConstants.ARM_VELOCITY, ArmConstants.ARM_ACCELERATION));
      // FF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

      pid.setTolerance(ArmConstants.SET_TOLERANCE);
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