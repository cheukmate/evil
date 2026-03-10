// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  // intake positions
  public static final Angle INTAKE_STOW_ANGLE = Degrees.of(0);
  public static final Angle INTAKE_DEPLOY_ANGLE = Degrees.of(125);

  public static boolean disableHAL = false;
  
  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class IDConstants{

    // Swerve Motor ID Constants.

    public static final int FRONTLEFT_DRIVE = 2; 
    public static final int FRONTLEFT_ANGLE = 1;
    public static final int FRONTLEFT_ENCODER = 21;

    public static final int FRONTRIGHT_DRIVE = 7;
    public static final int FRONTRIGHT_ANGLE = 6; 
    public static final int FRONTRIGHT_ENCODER = 23;

    public static final int BACKLEFT_DRIVE = 3;
    public static final int BACKLEFT_ANGLE = 4;
    public static final int BACKLEFT_ENCODER = 22;

    public static final int BACKRIGHT_DRIVE = 8;
    public static final int BACKRIGHT_ANGLE = 9;
    public static final int BACKRIGHT_ENCODER = 24;

    // Shooter ID Constants

    public static final int FLYWHEEL_MOTOR_MAIN_KRAKEN = 12;
    public static final int FLYWHEEL_MOTOR_FOLLOWER_KRAKEN = 13;

    public static final int SHOOTERHOOD = 38;

    public static final int INDEXER = 14;

    public static final int KICKER = 39;

    // Intake ID Constants

    public static final int PIVOT = 34;

    public static final int INTAKEWHEELS_FLEX_MAIN = 18;



    // Climber ID Constants

    public static final int CLIMBER_ID = 19;


  }

  public static final class Shooter {
    public static final AngularVelocity hubRPM = RPM.of(3000);

  }

  public static final class Hood {

    public static final Angle hubAngle = Degrees.of(30); // guessing
    public static final Angle startHoodAngle = Degrees.of(0); // well, yes
    public static final Angle lowerHoodAngle = Degrees.of(0); // well, yes
    public static final Angle higherHoodAngle = Degrees.of(25); // guessing

  }

}
