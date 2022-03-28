// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

/** Add your docs here. */
public class MkSwerveModule
{
    private MkSwerveDrive drive;
    private MkSwerveTurn turn;

    public MkSwerveModule(int[][] canid, double offset, NeutralMode[] mode, double[][] pid, boolean[][] inverted, int[] scurve, AbsoluteSensorRange range)
    {  
        this.drive = new MkSwerveDrive(canid[0][0], mode[0], pid[0], inverted[0][0], scurve[0]);
        this.turn = new MkSwerveTurn(canid[1], offset, mode[1], pid[1], inverted[1], scurve[1], range);
        this.turn.zeroMotorCANCoder();
    }

    public void setModule(double setpoint, ControlMode mode, double angle)
    {
        drive.setFalcon(mode, setpoint);
        turn.setFalcon(ControlMode.Position, angle);
    }

    public void setModuleMagic(double velocity, double accel)
    {
        drive.getFalcon().configMotionCruiseVelocity(velocity);
        drive.getFalcon().configMotionAcceleration(accel);
    }

    public MkSwerveDrive driveMotor()
    {
        return drive;
    }

    public MkSwerveTurn turnMotor()
    {
        return turn;
    }
}
