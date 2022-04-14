// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ToolShed;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKFALCON;

/** Add your docs here. */
public class MathFormulas {
/*
                                                  
                                                   E
                                               ~~~~~~~~~
                                            ~~     +     ~~     
                             /o----o\       ~~       + B     ~~       /o----o\
                             |  (F) |  (2) ~~        +        ~~ (1)  |  (F) |
                             \o----o/      ==========A==========      \o----o/ 
                                         \         |         /    
                                          \        |        /
                                           \       |       /
                                            \      |D     / 
                                             \     |     / 
                                              \  __|__  / 
                                               \/  C  \/
                                                \  |  /
                                                 \ | /
                                                  \|/
            
                             A = distanceA =
                             B = lengthB +
                             C = angle
                             D = radius |
                             E = circumference ~
                             F = robot
                             1 = starting position
                             2 = ending position
                             (diagram above isnt a hot air balloon fyi)
*/



    /**
     * Calculates a curved autonomous path's radius by using the distance between the starting and ending point and the distance between the middle of the path and the height of the angular path
     * @param distanceA
     * @param lengthB
     * @return Radius of the path
     */
    public static double calculateCircleRadius(double distanceA, double lengthB) {
        return ((Math.pow(distanceA, 2) / 4) + Math.pow(lengthB, 2)) * (1 / (2 * lengthB));
    }

    /**
     * Calculates a curved autonomous path's circumference/length by using the distance between the starting and ending point and the distance between the middle of the linear path and the max height of the angular path
     * @param distanceA
     * @param lengthB
     * @return Circumference of the linear path / distance of curved path
     * @see {@link #calculateCircleRadius(distanceA, lengthB)}
     */
    public static double calculateArcOfPath(double distanceA, double lengthB) {
        double radius = calculateCircleRadius(distanceA, lengthB);
        double theta = 2 * (Math.toDegrees((Math.asin((distanceA / (2 * radius))))));
        return (theta / 360) * (2 * (Constants.kPi * radius));
    }

    /**
     * Calculates a curved autonomous path's angle by using the distance between the starting and ending point and the distance between the middle of the path and the height of the angular path
     * @param distanceA
     * @param lengthB
     * @return Angle of the path (how much the angular motors have to turn in order to acheive this path)
     * @see {@link #calculateCircleRadius(distanceA, lengthB)}
     */
    public static double calculateAngleOfPath(double distanceA, double lengthB) {
        double radius = calculateCircleRadius(distanceA, lengthB);
        return 2 * (Math.toDegrees((Math.asin((distanceA / (2 * radius))))));
    }

    /**
   * (Falcon) native units to inches
   * <p>
   * specifically made for the driving motors
   * @param nativeUnits units of native
   * @return inches
   */
  public static double nativeToInches(double nativeUnits) {
    return (nativeUnits / (MKFALCON.oneEncoderRotation * MKDRIVE.greerRatio)) * MKDRIVE.wheelCircumference;
  }

  /**
   * inches to native units (Falcon)
   * <p>
   * specifically made for the driving motors
   * @param in inches
   * @return native units
   */
  public static double inchesToNative(double in) {
    return (in / MKDRIVE.wheelCircumference) * (MKFALCON.oneEncoderRotation * MKDRIVE.greerRatio);
  }

  /**
   * native units (1n/100s) to inches (1in/1s)
   * @param vel motor velocity (native units)
   * @return velocity of motors in inches
   */
  public static double nativePer100MstoInchesPerSec(double vel) {
    return 10 * nativeToInches(vel);
  }

  /**
   * inches (1in/1s) to native (1n/100s)
   * @param vel motor velocity (inches)
   * @return velocity of motors in native units
   */
  public static double inchesPerSecToUnitsPer100Ms(double vel) {
    return inchesToNative(vel) / 10;
  }

  /**
   * inches to meters
   * @param inches inches
   * @return meters
   */
  public static double inchesToMeters(double inches) {
    return Units.inchesToMeters(inches);
  }

  /** native units to meters
   * @param nativeUnits units of native
   * @return meters
   */
  public static double nativeToMeters(double nativeUnits) {
    return inchesToMeters(nativeToInches(nativeUnits));
  }

  /**
   * native units (1n/100s) to meters (1m/1s)
   * @param nativeUnits units of native
   * @return velocity of motors in meters
   */
  public static double nativePer100MsToMetersPerSec(double nativeUnits) {
    return inchesToMeters(nativePer100MstoInchesPerSec(nativeUnits));
  }

  /**
   * meters to inches
   * @param meters meters
   * @return inches
   */
  public static double metersToInches(double meters) {
    return Units.metersToInches(meters);
  }

  /**
   * meters (1m/1s) to native units (1n/100s)
   * @param meters meters
   * @return velocity of motors in native units
   */
  public static double metersPerSecondToNativeUnitsPer100Ms(double meters) {
    return inchesPerSecToUnitsPer100Ms(metersToInches(meters));
  }

  /**
   * (Falcon) native units to degrees
   * @param gimmeRots rotations of Falcon
   * @param greerRatio gear ratio 
   * @return degrees
   */
  public static double nativeToDegrees(double gimmeRots, double greerRatio)
  {
    return (gimmeRots * 360) / (greerRatio * MKFALCON.oneEncoderRotation);
  }

  /**
   * degrees to native units (Falcon)
   * @param gimmeDeg degrees
   * @param greerRatio gear ratio
   * @return native units
   */
  public static double degreesToNative(double gimmeDeg, double greerRatio)
  {
    return (gimmeDeg * MKFALCON.oneEncoderRotation * greerRatio) / 360;
  }

  /**
   * "Get the closest angle between the given angles."
   * @param a angle a
   * @param b angle b
   * @return angle closest between the two angles
   * @author team 6624
   */
  public static double closestAngle(double a, double b)
  {
        double dir = (b % 360.0) - (a % 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
  }
}
