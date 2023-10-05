
package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;

public class HoldArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Arm robotArm;

    private ArmFeedforward FF;
    private ProfiledPIDController PID;
    private DoubleSupplier setpoint;

    public HoldArmCommand(Arm robotArm, DoubleSupplier setpoint) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        addRequirements(robotArm);
    }

    @Override
    public void initialize() {

      FF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
      PID = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(ArmConstants.ARM_VELOCITY, ArmConstants.ARM_ACCELERATION));
      
      PID.setTolerance(ArmConstants.SET_TOLERANCE);
      PID.reset(robotArm.getPosition().getDegrees());

      SendableRegistry.setName(PID, "ArmSubsystem", "PID");

      System.out.println("Command HOLD ARM COMMAND has started");
    }
  
    @Override
    public void execute() {
        double PIDCalc = PID.calculate(robotArm.getPosition().getDegrees(), setpoint.getAsDouble());

        double calc = FF.calculate(
          Math.toRadians(robotArm.getPosition().getDegrees() - ArmConstants.ARM_PARALLEL_TO_GROUND),
          robotArm.getVelocity().getRadians()
        );

        if(robotArm.getShouldPID()) robotArm.armSpeedVolt(calc + PIDCalc);
        SmartDashboard.putNumber("Arm/Goal", PID.getGoal().position); 
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.armSpeedVolt(0);
      System.out.println("Command HOLD ARM COMMAND has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }

    public void resetPID() {
      PID.reset(robotArm.getPosition().getDegrees());
    }
  }