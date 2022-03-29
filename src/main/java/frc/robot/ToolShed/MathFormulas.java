// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ToolShed;

import frc.robot.Constants;

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
}
