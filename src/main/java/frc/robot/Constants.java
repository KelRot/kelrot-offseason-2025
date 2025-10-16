// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.Meters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean tuningMode = true; // Set to true to enable tuning mode, false to disable it
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class ArmConstants {
    public static final int MASTER_MOTOR_ID = 5;
    public static final int SLAVE_MOTOR_ID = 4;
    public static final double GEARING = 35.45142857142857;
    public static final double MAX_ANGLE = 90.0; // degrees
    public static final double MIN_ANGLE = 0.0; // degrees
    public static final double MAX_VELOCITY = 30.0; // degrees per second
    public static final double MAX_ACCELERATION = 90.0; // degrees per second squared
    public static final double ENCODER_DISTANCE_PER_PULSE = 4096;
    public static final double defaultAngle = 82.65; // degrees
  }

  public static final class ClimbConstants {
    public static final int openerNeoID = 2;
    public static final int closerNeoID = 3;
  }

  public static final class WristConstants {
    public static final int FalconID = 0;
    public static final int wheelMotorID = 5;
    public static final double rioKP = 0.03;
    public static final double rioKI = 0.0;
    public static final double rioKD = 0.00000098;
    public static final double MAX_ANGLE = 90.0; // degrees
    public static final double MIN_ANGLE = -90.0; // degrees
    public static final double MAX_VELOCITY = 100.0; // degrees per second
    public static final double MAX_ACCELERATION = 300.0; // degrees per second squared
    public static final double ENCODER_DISTANCE_PER_PULSE = 2048;
    public static final double defaultAngle = 0.0; // degrees
  }

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }
  public static class LevelAngles {
    public static final int Level1 = -75;
    public static final int Level2 = -70;
    public static final int FrontLevel3 = -20;
    public static final double BackLevel3 = 0.7;
    public static final double DefaultAngle = -82.65;
    public static final double DefaultAngleWrist = -12;
    public static final double BackLevel3Wrist = -2.6;
    public static final double BackAlgaeRemover = 0;
    public static final double BackAlgaeRemoverWrist = 0;
  }

  public static class LedConstants {
    public static final int kLedPort = 7; // PWM port on RoborIO
    public static final int kLedLength = 60; // led count
    public static final double kLedBlinkInterval = 500; // milliseconds
    public static final Distance kLedSpacing = Meters.of(1 / 60.0); // density of 60 LEDs per meter
    public static final int[][] kledGroups = {
        new int[] { 0, 59 }, // group 0
    };

  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }
}
