// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.CANID;
import frc.robot.Constants.MKCANCODER;
import frc.robot.Constants.MKTURN;
import frc.robot.ToolShed.FalconAlgorithims;


/** Add your docs here. */
public class MkSwerveTrain 
{
    private MkSwerveModule[] modules;
    public variables vars;

    private MkSwerveTrain()
    {
        modules = new MkSwerveModule[4];
        for(int i = 0; i < modules.length; i++)
        {
            modules[i] = new MkSwerveModule(CANID.MkTrainIds[i], MKCANCODER.offset[i]);
            modules[i].zeroTurn();
            modules[i].zeroDrive();
        }
        vars = new variables();
    }

    public static MkSwerveTrain getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateSwerve()
    {
        for(int i = 0; i < modules.length; i++)
        {
            vars.velNative[i] = modules[i].driveMotor().getVelocity();
            

            vars.posNative[i] = modules[i].driveMotor().getPosition();
            
            vars.velInch[i] = FalconAlgorithims.nativePer100MstoInchesPerSec(vars.velNative[i]);
            

            vars.posInch[i] = FalconAlgorithims.nativeToInches(vars.posNative[i]);
        
            
            vars.velMeters[i] = FalconAlgorithims.nativePer100MsToMetersPerSec(vars.velNative[i]);
            

            vars.posMeters[i] = FalconAlgorithims.nativeToMeters(vars.posNative[i]);

            vars.deg[i] = FalconAlgorithims.nativeToDegrees(modules[i].turnMotor().getPosition(), MKTURN.greerRatio);
        }
        vars.avgDistInches = (vars.posInch[0] + vars.posInch[1] + vars.posInch[2] + vars.posInch[3]) /4.0;
        vars.avgVelInches = (vars.velInch[0] + vars.velInch[1] + vars.velInch[2] + vars.velInch[3]) / 4.0;
        vars.avgVelNative = (vars.velNative[0] + vars.velNative[1] + vars.velNative[2] + vars.velNative[3]) / 4.0;
        vars.avgDeg = (vars.deg[0] + vars.deg[1] + vars.deg[2] + vars.deg[3]) / 4.0;
    }

    public MkSwerveModule[] getModules()
    {
        return modules;
    }

    public void setModuleTurn(double angle)
    {
        modules[0].turnMotor().setFalcon(ControlMode.Position, angle);
        modules[1].turnMotor().setFalcon(ControlMode.Position, angle);
        modules[2].turnMotor().setFalcon(ControlMode.Position, angle);
        modules[3].turnMotor().setFalcon(ControlMode.Position, angle);
    }

    private static class InstanceHolder
    {
        private static final MkSwerveTrain mInstance = new MkSwerveTrain();
    } 

    public static class variables
    {

        /**Distance variable for driving in autonomous*/
        public double straightDistance;

    /**Position of the driving motor in native units*/
    public double[] posNative;
       
    /**Position of the driving motor in inches*/
    public double[] posInch;
   
    /**Position of the driving motor in meters*/
    public double[] posMeters;
       
    /**Velocity of the driving motor in inches*/
    public double[] velInch;
   
    /**Velocity of the driving motor in native units*/
    public double[] velNative;
       
    /**Velocity of the driving motor in meters*/
    public double[] velMeters;
   
    /**Position of the turning motor in degrees*/
    public double[] deg;
   
    /**Driving motor values for autonomous*/
    public double[] output;
   
    /**Average velocity of driving motors in inches*/
    public double avgVelInches;
   
    /**Average velocity of driving motors in native units*/
    public double avgVelNative;
   
    /**Average distance of driving motors in inches*/
    public double avgDistInches;

    public double avgDeg;
   
    }
}
