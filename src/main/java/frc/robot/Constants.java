// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeConstants {
    public static final int INTAKE_ID = 20;
    public static final double INTAKE_SPEED = 0.5;
    public static final int CURRENT_LIMIT = 20;
  }

  public static class ArmConstants {
    public static final int LEFT_ARM_ID = 10;
    public static final int RIGHT_ARM_ID = 11;
    public static final int LEFT_ARM_CURRENT_LIMIT = 40;
    public static final int RIGHT_ARM_CURRENT_LIMIT = 40;
    public static final double ARM_PARALLEL_TO_GROUND = 91.0;
    public static final int SET_TOLERANCE = 2;
    public static final int ARM_VELOCITY = 500;
    public static final int ARM_ACCELERATION = 200;
  

    // public static final double kP = 0.007;
    // public static final double kI = 0.0;
    // public static final double kD = 0.0;

    // public static final double kS = 0.0;
    // public static final double kG = - 0.06;
    // public static final double kV = 0.0;
    // public static final double kA = 0.0;

    // public static final double xAxisOffset = - 91.0;

    // public static double To360Scope(double angle) {
    //   double val = angle + xAxisOffset;
    //   if(val < 0) val += 360;
    //   return 360 - val;
    // }
  }

  public static class DriveConstants {
    public static final int FRONT_RIGHT_ID = 2;
    public static final int BACK_RIGHT_ID = 3;
    public static final int FRONT_LEFT_ID = 4;
    public static final int BACK_LEFT_ID = 5;
    public static final int CURRENT_LIMIT = 40;
  }

  public static class CubeConstants {
    public static final double CUBE_IN_SPEED = 0.4;
    public static final double CUBE_OUT_SPEED = -1;

  }
}
