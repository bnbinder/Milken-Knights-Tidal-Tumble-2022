// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.Constants.MKFALCON;

/** Add your docs here. */
public class MkFalcon {
    private TalonFX falcon;

    public MkFalcon(int canid, NeutralMode mode, double[] pid, boolean inverted, int scurve)
    {
        falcon = new TalonFX(canid);
        falcon.configFactoryDefault();
        falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        falcon.setNeutralMode(mode);
        falcon.config_kP(0, pid[0]);
        falcon.config_kI(0, pid[1]);
        falcon.config_kD(0, pid[2]);
        falcon.config_kF(0, pid[3]);
        falcon.setInverted(inverted);      
        falcon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        falcon.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
        falcon.configVoltageCompSaturation(MKFALCON.voltComp);
        falcon.enableVoltageCompensation(true);
        falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
        falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
        falcon.configMotionSCurveStrength(scurve);
    }

    public double getVelocity()
    {
        return falcon.getSelectedSensorVelocity();
    }

    public double getPosition()
    {
        return falcon.getSelectedSensorPosition();
    }

    public void setPIDF(double[] pid)
    {
        falcon.config_kP(0, pid[0]);
        falcon.config_kI(0, pid[1]);
        falcon.config_kD(0, pid[2]);
        falcon.config_kF(0, pid[3]);
    }

    public void setMagicParams(double velocity, double accel)
    {
        falcon.configMotionCruiseVelocity(velocity);
        falcon.configMotionAcceleration(accel);
    }

    public void setFalcon(ControlMode mode, double setpoint)
    {
        falcon.set(mode, setpoint);
    }

    public void setFalconEncoder(double setpoint)
    {
        falcon.setSelectedSensorPosition(setpoint);
    }

    public int getCANID()
    {
        return falcon.getDeviceID();
    }

    public double getError()
    {
        return falcon.getClosedLoopError();
    }

    public TalonFX getFalcon()
    {
        return falcon;
    }
}
