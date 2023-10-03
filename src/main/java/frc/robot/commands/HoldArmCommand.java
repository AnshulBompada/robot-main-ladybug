
package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.ArmPID;

public class HoldArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Arm robotArm;

    private ArmFeedforward FF;
    private ProfiledPIDController PID;
    private DoubleSupplier setpoint;
    private double kS;
    private double kG;
    private double kV;
    private double kA;
    private double kP;
    private double kI;
    private double kD;

    public HoldArmCommand(Arm robotArm, DoubleSupplier setpoint) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        addRequirements(robotArm);
    }

    @Override
    public void initialize() {
      kS = 0;
      kG = -0.075;
      kV = 0;
      kA = 0;
      kP = 0.025;
      kI = 0;
      kD = 0;

      FF = new ArmFeedforward(kS, kG, kV, kA);
      PID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(ArmConstants.ARM_VELOCITY, ArmConstants.ARM_ACCELERATION));
      
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
          robotArm.getVelocity().getRadians() / 12
        );

        robotArm.armSpeed(calc + PIDCalc); 
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