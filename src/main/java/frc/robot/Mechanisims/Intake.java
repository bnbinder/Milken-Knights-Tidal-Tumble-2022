// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisims;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.CANID;
import frc.robot.Constants.INTAKE;
import frc.robot.Factory.Motor.MkFalcon;

/** Add your docs here. */
public class Intake 
{
    private MkFalcon intake = new MkFalcon(CANID.intakeCANID, INTAKE.intakeNeutralMode, INTAKE.pidf, false, INTAKE.scurve);
    private MkFalcon roller = new MkFalcon(CANID.rollerCANID, INTAKE.rollerNeutralMode, INTAKE.pidf, false, INTAKE.scurve);

    public static Intake getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void startIntake()
    {
        intake.getFalcon().configForwardSoftLimitEnable(true);
        intake.getFalcon().configForwardSoftLimitThreshold(INTAKE.maxIntakeNativePosition);
        intake.getFalcon().configReverseSoftLimitEnable(true);
        intake.getFalcon().configReverseSoftLimitThreshold(0);
    }

    public void setIntake(double setpoint, ControlMode mode)
    {
        intake.setFalcon(mode, setpoint);
    }

    public void setRoller(double setpoint, ControlMode mode)
    {
        roller.setFalcon(mode, setpoint);
    }

    private static class InstanceHolder
    {
        private static final Intake mInstance = new Intake();
    } 
}
