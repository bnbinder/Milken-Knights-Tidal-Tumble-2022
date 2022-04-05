// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ToolShed;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.navx;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Constants.MKTURN;
import frc.robot.Factory.Motor.MkSwerveModule;
import frc.robot.Mechanisims.MkSwerveTrain;

/** Add your docs here. */
public class SwerveAlgorithims {

    private MkSwerveModule[] modules;
    private variables var;
    private double hP = 0.001, hI = 0.0001, hD = hP * 0.1;
    private double hIntegral, hDerivative, hPreviousError, hError;

    private SwerveAlgorithims()
    {
        modules = MkSwerveTrain.getInstance().getModules();
        var = new variables();
        var.mod1 = new double[2];
        var.mod2 = new double[2];
        var.mod3 = new double[2];
        var.mod4 = new double[2];
    }

    public static SwerveAlgorithims getInstance()
    {
        return InstanceHolder.mInstance;
    }

    /**
     * See <a href="https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383">this thread</a>
     * for more information.
     * <p>
     * Note - this function uses 180 minus yaw due to the positioning of our navx.
     * @param FWD Forward axis of controller
     * @param STR Strafe axis of controller
     * @param RCW Rotational axis of controller
     * @author Ether
     */
    public void etherSwerve(double FWD, double STR, double RCW)
    {
        var.yaw = navx.getInstance().getNavxYaw();
        var.temp = FWD * Math.cos(Math.toRadians(var.yaw)) + STR * Math.sin(Math.toRadians(var.yaw));
        STR = -FWD * Math.sin(Math.toRadians(var.yaw)) + STR * Math.cos(Math.toRadians(var.yaw));
        FWD = var.temp;

        //SmartDashboard.putNumber("frd", FWD);
        //SmartDashboard.putNumber("str", STR);

        var.A = STR - RCW*(MKTRAIN.L/MKTRAIN.R);
        var.B = STR + RCW*(MKTRAIN.L/MKTRAIN.R);
        var.C = FWD - RCW*(MKTRAIN.W/MKTRAIN.R);
        var.D = FWD + RCW*(MKTRAIN.W/MKTRAIN.R);
        
        var.mod1[0] = Math.sqrt((Math.pow(var.B, 2)) + (Math.pow(var.C, 2)));      var.mod1[1] = Math.atan2(var.B,var.C)*180/Constants.kPi;
        var.mod2[0] = Math.sqrt((Math.pow(var.B, 2)) + (Math.pow(var.D, 2)));      var.mod2[1] = Math.atan2(var.B,var.D)*180/Constants.kPi;
        var.mod3[0] = Math.sqrt((Math.pow(var.A, 2)) + (Math.pow(var.D, 2)));      var.mod3[1] = Math.atan2(var.A,var.D)*180/Constants.kPi;
        var.mod4[0] = Math.sqrt((Math.pow(var.A, 2)) + (Math.pow(var.C, 2)));      var.mod4[1] = Math.atan2(var.A,var.C)*180/Constants.kPi; 

        var.max=var.mod1[0]; if(var.mod2[0]>var.max)var.max=var.mod2[0]; if(var.mod3[0]>var.max)var.max=var.mod3[0]; if(var.mod4[0]>var.max)var.max=var.mod4[0];
        if(var.max>1){var.mod1[0]/=var.max; var.mod2[0]/=var.max; var.mod3[0]/=var.max; var.mod4[0]/=var.max;}


        var.mod1 = SwerveAlgorithims.setDirection(modules[1].turnMotor().getPosition(), var.mod1);
        var.mod2 = SwerveAlgorithims.setDirection(modules[0].turnMotor().getPosition(), var.mod2);
        var.mod3 = SwerveAlgorithims.setDirection(modules[2].turnMotor().getPosition(), var.mod3);
        var.mod4 = SwerveAlgorithims.setDirection(modules[3].turnMotor().getPosition(), var.mod4);

        SmartDashboard.putNumber("wa1", FalconAlgorithims.degreesToNative(var.mod1[1], MKTURN.greerRatio));
        modules[1].setModule(var.mod1[0], ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.mod1[1], MKTURN.greerRatio));
        modules[0].setModule(var.mod2[0], ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.mod2[1], MKTURN.greerRatio));
        modules[2].setModule(var.mod3[0], ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.mod3[1], MKTURN.greerRatio));
        modules[3].setModule(var.mod4[0], ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.mod4[1], MKTURN.greerRatio));
        //TODO velocity might break the drive pidf
    }



   /**
     * decides whether a driving motor should flip based on where the angular motor's setpoint is.
     * @param position position of the motor
     * @param setpoint setpoint for the motor
     * @return returns best angle of travel for the angular motor, as well as the flip value for the driving motor (as an array so it can return two things in one instead of two seperatly)
     * @author team 6624
     */
    public static double[] setDirection(double position, double[] mod)
    {
        double currentAngle = FalconAlgorithims.nativeToDegrees(position, MKTURN.greerRatio);
        // find closest angle to setpoint
        double setpointAngle = FalconAlgorithims.closestAngle(currentAngle, mod[1]);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = FalconAlgorithims.closestAngle(currentAngle, mod[1] + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            return new double[] {mod[0],(currentAngle + setpointAngle)};
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            return new double[] {Math.abs(mod[0]) * -1.0, (currentAngle + setpointAngleFlipped)};
        }
    }

    //programming done right
    public double headerStraighter(double hSetpoint)
    {
        if(hSetpoint != 361)
        {
            hError = hSetpoint -  navx.getInstance().getNavxYaw();// Error = Target - Actual
            hIntegral += (hError*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            hDerivative = (hError - hPreviousError) / .02;
            return hP*hError + hI*hIntegral + hD*hDerivative;
        }
        else
        {
            return 0;
        }
    }

    private static class InstanceHolder
    {
        private static final SwerveAlgorithims mInstance = new SwerveAlgorithims();
    } 

    public static class variables 
    {
        public double temp;
        public double yaw;
        public double A;
        public double B;
        public double C;
        public double D;
        public double mod1[];
        public double mod2[];
        public double mod3[];
        public double mod4[];
        public double max;
    }
}
