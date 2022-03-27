// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Add your docs here. */
public class MkSwerveTurn extends MkFalcon 
{
    public MkSwerveTurn(int canid, NeutralMode mode, double[] pid, boolean inverted, int scurve)
    {
        super(canid, mode, pid, inverted, scurve);
    }
}
