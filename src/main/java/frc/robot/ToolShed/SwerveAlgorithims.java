// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ToolShed;

import frc.robot.navx;
import frc.robot.Constants.MKTURN;
import frc.robot.ToolShed.FalconAlgorithims;


/** Add your docs here. */
public class SwerveAlgorithims {
    public static double[] setDirection(double position, double setpoint)
    {
        double currentAngle = FalconAlgorithims.nativeToDegrees(position, MKTURN.greerRatio);
        // find closest angle to setpoint
        double setpointAngle = FalconAlgorithims.closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = FalconAlgorithims.closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            return new double[] {(currentAngle + setpointAngle), 1.0};
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            return new double[] {(currentAngle + setpointAngleFlipped), -1.0};
        }
    }

    double hP = 0.001, hI = 0.0001, hD = hP * 0.1;
    double hIntegral, hDerivative, hPreviousError, hError;

    //programming done right
    public double headerStraighter(double hSetpoint)
    {
        hError = hSetpoint -  navx.getInstance().getNavxYaw();// Error = Target - Actual
        hIntegral += (hError*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        hDerivative = (hError - hPreviousError) / .02;
        return hP*hError + hI*hIntegral + hD*hDerivative;
    }
}
