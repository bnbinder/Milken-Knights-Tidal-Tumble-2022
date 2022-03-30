// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Motor;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MKCANCODER;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKTURN;
import frc.robot.ToolShed.FalconAlgorithims;

/** Add your docs here. */
public class MkSwerveModule
{
    private MkFalcon drive;
    private MkFalcon turn;
    private MkCANCoder encoder;
    private SimpleMotorFeedforward driveFeed;
    private double setpoint = 0;

    public MkSwerveModule(int[] canid, double offset)
    {  
        this.drive = new MkFalcon(canid[0], MKDRIVE.mode, MKDRIVE.pidf, MKDRIVE.inverted, MKDRIVE.scurve);
        this.turn = new MkFalcon(canid[1], MKTURN.mode, MKTURN.pidf, MKTURN.inverted, MKTURN.scurve);
        this.encoder = new MkCANCoder(canid[2], offset, MKCANCODER.inverted, MKCANCODER.range);
        this.driveFeed = new SimpleMotorFeedforward(MKDRIVE.kS, MKDRIVE.kV, MKDRIVE.kA);
    }

    public void setModule(double setpoint, ControlMode mode, double angle)
    {
        if(mode == ControlMode.Velocity)
        {
            this.setpoint = setpoint;
            setpoint += driveFeed.calculate(drive.getVelocity(), setpoint, 0.02); //TODO might break code
        }
        drive.setFalcon(mode, setpoint);
        turn.setFalcon(ControlMode.Position, angle);
    }

    public double getFF()
    {
       return driveFeed.calculate(drive.getVelocity(), this.setpoint, 0.02); 
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(FalconAlgorithims.nativePer100MsToMetersPerSec(drive.getVelocity()), new Rotation2d(Math.toRadians(encoder.getAbsPosition())));
      }
      
      public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(encoder.getAbsPosition())));
            drive.setFalcon(ControlMode.Velocity, FalconAlgorithims.metersPerSecondToNativeUnitsPer100Ms(state.speedMetersPerSecond));  // + driveFeedforward);
            turn.setFalcon(ControlMode.Position, FalconAlgorithims.degreesToNative(state.angle.getDegrees(), MKTURN.greerRatio));  // + turnFeedforward);
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
