// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import frc.robot.Constants.MKCANCODER;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Constants.MKTURN;

/** Add your docs here. */
public class MkSwerveModule
{
    private MkFalcon drive;
    private MkFalcon turn;
    private MkCANCoder encoder;

    public MkSwerveModule(int[] canid, double offset)
    {  
        this.drive = new MkFalcon(canid[0], MKDRIVE.mode, MKDRIVE.pidf, MKDRIVE.inverted, MKDRIVE.scurve);
        this.turn = new MkFalcon(canid[1], MKTURN.mode, MKTURN.pidf, MKTURN.inverted, MKTURN.scurve);
        this.encoder = new MkCANCoder(canid[2], offset, MKCANCODER.inverted, MKCANCODER.range);
    }

    public void setModule(double setpoint, ControlMode mode, double angle)
    {
        drive.setFalcon(mode, setpoint);
        turn.setFalcon(ControlMode.Position, angle);
    }

    public void zeroTurn()
    {
        turn.setFalconEncoder(encoder.getAbsPosition());
    }

    public void zeroDrive()
    {
        drive.setFalconEncoder(0);
    }

    public MkFalcon driveMotor()
    {
        return drive;
    }

    public MkFalcon turnMotor()
    {
        return turn;
    }

    public MkCANCoder encoder()
    {
        return encoder;
    }
}
