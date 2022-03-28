// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ToolShed;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.navx;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Constants.MKTURN;
import frc.robot.Factory.Motor.MkSwerveModule;
import frc.robot.Factory.Motor.MkSwerveTrain;

/** Add your docs here. */
public class Ether extends SwerveAlgorithims
{
    private MkSwerveModule[] modules;
    private variables var;

    private Ether()
    {
        modules = MkSwerveTrain.getInstance().getModules();
        var = new variables();
    }

    public static Ether getInstance()
    {
        return InstanceHolder.mInstance;
    }

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
        
        var.ws1 = Math.sqrt((Math.pow(var.B, 2)) + (Math.pow(var.C, 2)));      var.wa1 = Math.atan2(var.B,var.C)*180/Constants.kPi;
        var.ws2 = Math.sqrt((Math.pow(var.B, 2)) + (Math.pow(var.D, 2)));      var.wa2 = Math.atan2(var.B,var.D)*180/Constants.kPi;
        var.ws3 = Math.sqrt((Math.pow(var.A, 2)) + (Math.pow(var.D, 2)));      var.wa3 = Math.atan2(var.A,var.D)*180/Constants.kPi;
        var.ws4 = Math.sqrt((Math.pow(var.A, 2)) + (Math.pow(var.C, 2)));      var.wa4 = Math.atan2(var.A,var.C)*180/Constants.kPi; 

        var.max=var.ws1; if(var.ws2>var.max)var.max=var.ws2; if(var.ws3>var.max)var.max=var.ws3; if(var.ws4>var.max)var.max=var.ws4;
        if(var.max>1){var.ws1/=var.max; var.ws2/=var.max; var.ws3/=var.max; var.ws4/=var.max;}


        var.mod1 = SwerveAlgorithims.setDirection(modules[1].turnMotor().getPosition(), var.wa1);
        var.mod2 = SwerveAlgorithims.setDirection(modules[0].turnMotor().getPosition(), var.wa2);
        var.mod3 = SwerveAlgorithims.setDirection(modules[2].turnMotor().getPosition(), var.wa3);
        var.mod4 = SwerveAlgorithims.setDirection(modules[3].turnMotor().getPosition(), var.wa4);

        var.wa1 = var.mod1[0];
        var.wa2 = var.mod2[0];
        var.wa3 = var.mod3[0];
        var.wa4 = var.mod4[0];

        var.ws1 *= var.mod1[1];
        var.ws2 *= var.mod2[1];
        var.ws3 *= var.mod3[1];
        var.ws4 *= var.mod4[1];

        modules[1].setModule(var.ws2, ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.wa2, MKTURN.greerRatio));
        modules[0].setModule(var.ws1, ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.wa1, MKTURN.greerRatio));
        modules[2].setModule(var.ws3, ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.wa3, MKTURN.greerRatio));
        modules[3].setModule(var.ws4, ControlMode.PercentOutput, FalconAlgorithims.degreesToNative(var.wa4, MKTURN.greerRatio));
    }

    private static class InstanceHolder
    {
        private static final Ether mInstance = new Ether();
    } 

    public static class variables 
    {
        public double temp;
        public double yaw;
        public double A;
        public double B;
        public double C;
        public double D;
        public double wa1;
        public double wa2;
        public double wa3;
        public double wa4;
        public double ws1;
        public double ws2;
        public double ws3;
        public double ws4;
        public double mod1[];
        public double mod2[];
        public double mod3[];
        public double mod4[];
        public double max;
    }
}
