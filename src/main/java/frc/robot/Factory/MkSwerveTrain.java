// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.navx;
import frc.robot.Constants.CANID;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Constants.MKTURN;
import frc.robot.ToolShed.FalconAlgorithims;
import frc.robot.ToolShed.SwerveAlgorithims;


/** Add your docs here. */
public class MkSwerveTrain 
{
    private MkSwerveModule[] modules = new MkSwerveModule[4];
    private navx navX;
    private variables vars;

    public static MkSwerveTrain getInstance()
    {
        return InstanceHolder.mInstance;
    }

    private MkSwerveTrain()
    {
        for(int i = 0; i < modules.length; i++)
        {
            modules[i] = new MkSwerveModule(CANID.MkTrainIds[i], MKTRAIN.mode[i], MKTRAIN.pidf[i], MKTRAIN.inverted[i], MKTRAIN.scurve[i]);
        }
        navX = navx.getInstance();
        vars = new variables();
    }

    public void etherSwerve(double FWD, double STR, double RCW)
    {
        vars.yaw = navX.getNavxYaw();
        vars.temp = FWD * Math.cos(Math.toRadians(vars.yaw)) + STR * Math.sin(Math.toRadians(vars.yaw));
        STR = -FWD * Math.sin(Math.toRadians(vars.yaw)) + STR * Math.cos(Math.toRadians(vars.yaw));
        FWD = vars.temp;

        //SmartDashboard.putNumber("frd", FWD);
        //SmartDashboard.putNumber("str", STR);

        vars.A = STR - RCW*(MKTRAIN.L/MKTRAIN.R);
        vars.B = STR + RCW*(MKTRAIN.L/MKTRAIN.R);
        vars.C = FWD - RCW*(MKTRAIN.W/MKTRAIN.R);
        vars.D = FWD + RCW*(MKTRAIN.W/MKTRAIN.R);
        
        vars.ws1 = Math.sqrt((Math.pow(vars.B, 2)) + (Math.pow(vars.C, 2)));      vars.wa1 = Math.atan2(vars.B,vars.C)*180/Constants.kPi;
        vars.ws2 = Math.sqrt((Math.pow(vars.B, 2)) + (Math.pow(vars.D, 2)));      vars.wa2 = Math.atan2(vars.B,vars.D)*180/Constants.kPi;
        vars.ws3 = Math.sqrt((Math.pow(vars.A, 2)) + (Math.pow(vars.D, 2)));      vars.wa3 = Math.atan2(vars.A,vars.D)*180/Constants.kPi;
        vars.ws4 = Math.sqrt((Math.pow(vars.A, 2)) + (Math.pow(vars.C, 2)));      vars.wa4 = Math.atan2(vars.A,vars.C)*180/Constants.kPi; 

        vars.max=vars.ws1; if(vars.ws2>vars.max)vars.max=vars.ws2; if(vars.ws3>vars.max)vars.max=vars.ws3; if(vars.ws4>vars.max)vars.max=vars.ws4;
        if(vars.max>1){vars.ws1/=vars.max; vars.ws2/=vars.max; vars.ws3/=vars.max; vars.ws4/=vars.max;}


        vars.mod1 = SwerveAlgorithims.setDirection(modules[1].turnMotor().getPosition(), vars.wa1);
        vars.mod2 = SwerveAlgorithims.setDirection(modules[0].turnMotor().getPosition(), vars.wa2);
        vars.mod3 = SwerveAlgorithims.setDirection(modules[2].turnMotor().getPosition(), vars.wa3);
        vars.mod4 = SwerveAlgorithims.setDirection(modules[3].turnMotor().getPosition(), vars.wa4);

        vars.wa1 = vars.mod1[0];
        vars.wa2 = vars.mod2[0];
        vars.wa3 = vars.mod3[0];
        vars.wa4 = vars.mod4[0];

        vars.ws1 *= vars.mod1[1];
        vars.ws2 *= vars.mod2[1];
        vars.ws3 *= vars.mod3[1];
        vars.ws4 *= vars.mod4[1];

        modules[1].turnMotor().setFalcon(ControlMode.Position, FalconAlgorithims.degreesToNative(vars.wa1, MKTURN.greerRatio)); 
        modules[0].turnMotor().setFalcon(ControlMode.Position, FalconAlgorithims.degreesToNative(vars.wa2, MKTURN.greerRatio)); 
        modules[2].turnMotor().setFalcon(ControlMode.Position, FalconAlgorithims.degreesToNative(vars.wa3, MKTURN.greerRatio)); 
        modules[3].turnMotor().setFalcon(ControlMode.Position, FalconAlgorithims.degreesToNative(vars.wa4, MKTURN.greerRatio)); 

        modules[1].driveMotor().setFalcon(ControlMode.PercentOutput, vars.ws2);
        modules[0].driveMotor().setFalcon(ControlMode.PercentOutput, vars.ws1);
        modules[2].driveMotor().setFalcon(ControlMode.PercentOutput, vars.ws3);
        modules[3].driveMotor().setFalcon(ControlMode.PercentOutput, vars.ws4);
    }

    private static class InstanceHolder
    {
        private static final MkSwerveTrain mInstance = new MkSwerveTrain();
    } 

    public static class variables
    {
        public double yaw;
        public double temp;
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
