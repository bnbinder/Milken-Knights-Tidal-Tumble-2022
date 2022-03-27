// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

/** Add your docs here. */
public class MkSwerveTurn extends MkFalcon 
{
    private MkCANCoder coder;

    public MkSwerveTurn(int[] canid, double offset, NeutralMode mode, double[] pid, boolean[] inverted, int scurve, AbsoluteSensorRange range)
    {
        super(canid[0], mode, pid, inverted[0], scurve);
        coder = new MkCANCoder(canid[1], offset, inverted[1], range);
    }

    public void zeroMotorCANCoder()
    {
        super.setFalconEncoder(coder.getAbsPosition());
    }
}
