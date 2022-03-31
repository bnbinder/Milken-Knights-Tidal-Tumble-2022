// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisims;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.CANID;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Factory.Motor.MkFalcon;

/** Add your docs here. */
public class Elevator 
{
    private MkFalcon elevator = new MkFalcon(CANID.elevatorCANID, ELEVATOR.elevatorNeutralMode, ELEVATOR.pidf, false, ELEVATOR.scurve);

    public static Elevator getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void setElevator(double setpoint, ControlMode mode)
    {
        elevator.setFalcon(mode, setpoint);
    }

    private static class InstanceHolder
    {
        private static final Elevator mInstance = new Elevator();
    } 
}
