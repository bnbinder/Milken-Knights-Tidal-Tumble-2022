// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisims;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.CANID;
import frc.robot.Constants.SHOOTER;
import frc.robot.Factory.Motor.MkFalcon;

/** Add your docs here. */
public class Shooter 
{
    private MkFalcon shootLeft = new MkFalcon(CANID.leftShooterCANID, SHOOTER.leftShootNeutralMode, SHOOTER.pidf, SHOOTER.isLeftInverted, SHOOTER.scurve);
    private MkFalcon shootRight = new MkFalcon(CANID.rightShooterCANID, SHOOTER.rightShootNeutralMode, SHOOTER.pidf, !SHOOTER.isLeftInverted, SHOOTER.scurve);

    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void setLeftShoot(double setpoint, ControlMode mode)
    {
        shootLeft.setFalcon(mode, setpoint);
    }

    public void setRightShoot(double setpoint, ControlMode mode)
    {
        shootRight.setFalcon(mode, setpoint);
    }

    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    } 
}
