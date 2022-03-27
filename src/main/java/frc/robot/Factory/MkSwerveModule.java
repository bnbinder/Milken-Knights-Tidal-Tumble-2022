// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Add your docs here. */
public class MkSwerveModule
{
    private MkSwerveDrive drive;
    private MkSwerveTurn turn;

    public MkSwerveModule(int[] canid, NeutralMode[] mode, double[][] pid, boolean[] inverted, int[] scurve)
    {  
        this.drive = new MkSwerveDrive(canid[1], mode[1], pid[1], inverted[1], scurve[1]);
        this.turn = new MkSwerveTurn(canid[0], mode[0], pid[0], inverted[0], scurve[0]);
    }

    public MkSwerveDrive driveMotor()
    {
        return drive;
    }

    public MkSwerveTurn turnMotor()
    {
        return turn;
    }

    public void setModule(double setpoint, ControlMode mode, double angle)
    {
        drive.setFalcon(mode, setpoint);
        turn.setFalcon(ControlMode.Position, angle);
    }
}
