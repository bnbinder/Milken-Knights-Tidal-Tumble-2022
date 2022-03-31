// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisims;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.CANID;
import frc.robot.Constants.CLIMBER;
import frc.robot.Factory.Motor.MkFalcon;

/** Add your docs here. */
public class Climber {
    private MkFalcon leftClimb = new MkFalcon(CANID.leftClimberCANID, CLIMBER.leftClimbNeutralMode, CLIMBER.pidf, CLIMBER.isLeftInverted, CLIMBER.scurve);
    private MkFalcon rightClimb = new MkFalcon(CANID.rightClimberCANID, CLIMBER.rightClimbNeutralMode, CLIMBER.pidf, !CLIMBER.isLeftInverted, CLIMBER.scurve);
    
    public static Climber getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void startClimb()
    {
        leftClimb.getFalcon().configForwardSoftLimitEnable(true);
        leftClimb.getFalcon().configForwardSoftLimitThreshold(CLIMBER.maxNativePosition);
        leftClimb.getFalcon().configReverseSoftLimitEnable(true);
        leftClimb.getFalcon().configReverseSoftLimitThreshold(0);

        rightClimb.getFalcon().configForwardSoftLimitEnable(true);
        rightClimb.getFalcon().configForwardSoftLimitThreshold(CLIMBER.maxNativePosition);
        rightClimb.getFalcon().configReverseSoftLimitEnable(true);
        rightClimb.getFalcon().configReverseSoftLimitThreshold(0);
    }

    public void setLeftClimb(double setpoint, ControlMode mode)
    {
        leftClimb.setFalcon(mode, setpoint);
    }

    public void setRightClimb(double setpoint, ControlMode mode)
    {
        rightClimb.setFalcon(mode, setpoint);
    }

    public void setClimb(double setpoint, ControlMode mode)
    {
        leftClimb.setFalcon(mode, setpoint);
        rightClimb.setFalcon(mode, setpoint);
    }

    public void setLeftEncoder(double setpoint)
    {
        leftClimb.setFalconEncoder(setpoint);
    }

    public void setRightEncoder(double setpoint)
    {
        leftClimb.setFalconEncoder(setpoint);
    }

    private static class InstanceHolder
    {
        private static final Climber mInstance = new Climber();
    } 

}
