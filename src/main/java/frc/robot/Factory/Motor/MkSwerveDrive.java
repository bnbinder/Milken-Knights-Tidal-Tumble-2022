// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.MKDRIVE;

/** Add your docs here. */
public class MkSwerveDrive extends MkFalcon
{
    private SimpleMotorFeedforward ff;

    public MkSwerveDrive(int canid, NeutralMode mode, double[] pid, boolean inverted, int scurve)
    {
        super(canid, mode, pid, inverted, scurve);
        ff = new SimpleMotorFeedforward(MKDRIVE.kS, MKDRIVE.kV, MKDRIVE.kA);
    }

    public double getFeedForward(double current, double next)
    {
        return ff.calculate(current, next, 0.02);
    }
}
