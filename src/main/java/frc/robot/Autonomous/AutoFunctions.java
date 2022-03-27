// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import frc.robot.Factory.MkSwerveTrain;

/** Add your docs here. */
public class AutoFunctions {
    public MkSwerveTrain train;

    private AutoFunctions()
    {

    }

    public static AutoFunctions getInstance()
    {
        return InstanceHolder.mInstance;
    }

    /**Restarts distance*/
    public void autoTurnSet()
    {
        currentDistance = 0;
    }

    /**
     * Using the {@link #swerveAutonomousEther} and motion magic, an autonomous angled path of motion can be achieved
     * @param totalDistance Length of curved path
     * @param thetaTurn Angle of curved path
     * @param RCWauto [-1, 1] For spinny, 0 for no spinny
     * @param mode Curve or Straight
     * @param turny Specific or Infinite
     * @param turnyAuto (if using specific for turny) angle that robot tries to keep when moving
     * @see {@link #swerveAutonomousEther(FWD, STR, RCW)}
     * @see {@link #updateMagicStraight()}
    */
    public void autoTurnUpdate(double totalDistance, double thetaTurn, double RCWauto, ETHERAUTO mode, ETHERRCW turny, double turnyAuto)
    {
        double RCWtemp = RCWauto;
        currentDistance = 
            (MkUtil.nativeToInches(topDriveLeft.getSelectedSensorPosition()) +
            MkUtil.nativeToInches(topDriveRight.getSelectedSensorPosition()) +
            MkUtil.nativeToInches(bottomDriveLeft.getSelectedSensorPosition()) + 
            MkUtil.nativeToInches(bottomDriveRight.getSelectedSensorPosition())) / 4;
        if(mode == ETHERAUTO.Curve)
        {
            FWDauto = Math.cos((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)) * Constants.kPi / 180);
            STRauto = Math.sin((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)) * Constants.kPi / 180);
            SmartDashboard.putNumber("STRauto", STRauto);
            SmartDashboard.putNumber("FWDauto", FWDauto);
        }
        else if(mode == ETHERAUTO.Straight)
        {
            FWDauto = Math.cos(thetaTurn);
            STRauto = Math.sin(thetaTurn);
            SmartDashboard.putNumber("STRauto", STRauto);
            SmartDashboard.putNumber("FWDauto", FWDauto);
        }
        if(turny == ETHERRCW.Specific)
        {
            RCWtemp = headerStraighter(turnyAuto);
        }
        swerveAutonomousEther(-FWDauto, STRauto, RCWtemp);
        //SmartDashboard.putNumber("AUTONASX", getNavx());
        SmartDashboard.putNumber("jihngnhjing", headerStraighter(turnyAuto));

        //swerveAutonomousEther(FWDauto, -STRauto, RCWtemp);
/*
        topTurnLeft.set(ControlMode.PercentOutput, topTurnLeftCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, topTurnRightCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, bottomTurnLeftCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, bottomTurnRightCalculateNative(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
 */
    }

    /**
     * Returns state of auto turn
     * @param  totalDistance Length of curved path (the same distance set in the {@link #setMagicStraight} function)
     * @return True if turning is done
     * @see {@link #setMagicStraight(setpoint)}
     */
    public boolean autoTurnIsDone(double totalDistance)
    {
        return Math.abs(totalDistance - currentDistance) < 0.5 && Math.abs(avgVelInches) < 0.1;
    }


    /**
     * Resets drive motors and sets motion magic velocity, acceleration, and distance
     * @param setpoint Distance (inches)
     * @param magicVelo Motion magic maximum velocity
     * @param magicAccel Motion magic maximum acceleration
     */
    public void setMagicStraight(double setpoint, double magicVelo, double magicAccel)
    {
        resetDrive();
        distance = setpoint;
        topDriveLeft.configMotionCruiseVelocity(magicVelo);
        topDriveRight.configMotionCruiseVelocity(magicVelo);
        bottomDriveLeft.configMotionCruiseVelocity(magicVelo);
        bottomDriveRight.configMotionCruiseVelocity(magicVelo);

        topDriveLeft.configMotionAcceleration(magicAccel);
        topDriveRight.configMotionAcceleration(magicAccel);
        bottomDriveLeft.configMotionAcceleration(magicAccel);
        bottomDriveRight.configMotionAcceleration(magicAccel);
    }

    /**
     * Resets turn motors and sets motion magic velocity, acceleration, and distance
     * @param setpoint Distance or angle idk anymore (inches, and the same distance set in the {@link #setMagicStraight} function). however, if you are turning without driving, use native units instead of inches
     * @see {@link #setMagicStraight(setpoint)}
     */
    public void setMagicTurn(double setpoint)
    {
        turnDistance = setpoint;
        topTurnLeft.configMotionCruiseVelocity(DRIVE.magicVelo);
        topTurnRight.configMotionCruiseVelocity(DRIVE.magicVelo);
        bottomTurnLeft.configMotionCruiseVelocity(DRIVE.magicVelo);
        bottomTurnRight.configMotionCruiseVelocity(DRIVE.magicVelo);

        topTurnLeft.configMotionAcceleration(DRIVE.magicAccel);
        topTurnRight.configMotionAcceleration(DRIVE.magicAccel);
        bottomTurnLeft.configMotionAcceleration(DRIVE.magicAccel);
        bottomTurnRight.configMotionAcceleration(DRIVE.magicAccel);
    }

    /**
     * Updates turn motors with magic
     * @param totalDistance The same distance set in the {@link #setMagicStraight} function
     * @see {@link #setMagicStraight(setpoint)}
     */
    public void updateMagicTurn(double totalDistance)
    {
        currentDistance = 
        (MkUtil.nativeToInches(topDriveLeft.getSelectedSensorPosition()) +
        MkUtil.nativeToInches(topDriveRight.getSelectedSensorPosition()) +
        MkUtil.nativeToInches(bottomDriveLeft.getSelectedSensorPosition()) + 
        MkUtil.nativeToInches(bottomDriveRight.getSelectedSensorPosition())) / 4;

        topTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        topTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        bottomTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        bottomTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative((-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)), TURN.greerRatio));
        //SmartDashboard.putNumber("currentdist", currentDistance);
        //SmartDashboard.putNumber("WORK", (-1 * turnDistance) + (2 * ((currentDistance/totalDistance)*turnDistance)));
    }

    /**
     * Updates turn motors with magic if you arent turning while driving and have set the setpoint in {@link #setMagicTurn} to native units instead of a distance (inches)
     * @see {@link #setMagicTurn(setpoint)}
     */
    public void updateMagicTurnAlone()
    {
        topTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
        topTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
        bottomTurnLeft.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
        bottomTurnRight.set(ControlMode.MotionMagic, MkUtil.degreesToNative(turnDistance, TURN.greerRatio));
    }

 /**Updates drive motors with magic*/
    public void updateMagicStraight()
    {
        topDriveLeft.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        topDriveRight.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        bottomDriveLeft.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        bottomDriveRight.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));

        leftTopOutput = MkUtil.inchesToNative(distance);
        rightTopOutput = MkUtil.inchesToNative(distance);
        leftBottomOutput = MkUtil.inchesToNative(distance);
        rightBottomOutput = MkUtil.inchesToNative(distance);

        //SmartDashboard.putNumber("dist", distance);
        //SmartDashboard.putNumber("leftout", leftTopOutput);
    }

    /**
     * Returns state of motion magic
     * @return True if motion magic is done
     */
    public boolean isMagicStraightDone()
    {
        double err = distance - avgDistInches;              //TODO 0.1?? 
        return Math.abs(err) < 0.1 && Math.abs(avgVelInches) < 0.1; //0.5, 0.5
    }

    /**
     * same as {@link #isMagicStraightDone} but you can customize the distance variable
     * @param inchesEther distance of the ether
     * @return True if ether is done
     */
    public boolean isEtherMoveDone(double inchesEther)
    {
        double err = inchesEther - avgDistInches;              //TODO 0.1?? 
        return Math.abs(err) < 0.1 && Math.abs(avgVelInches) < 0.1; //0.5, 0.5
    }

    /**
     * Returns the state of percent turning (use if using {@link #updateMagicTurnAlone} and using native units instead of inches in {@link #setMagicTurn})
     * @return True if percent turning is done
     * @see {@link #updateMagicTurnAlone()}
     * @see {@link #setMagicTurn(setpoint)}
     */
    public boolean percentTurnDone()
    {
        double err = turnDistance - ((MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio) + MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio) + MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio) + MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio)) /4.0);
        return Math.abs(err) < 0.5;
    }

    private static class InstanceHolder
    {
        private static final AutoFunctions mInstance = new AutoFunctions();
    } 

    /**Mode of the ether auto's path*/
    public enum ETHERAUTO
    {
        Straight, Curve
    }

    /**Mode of the ether auto's turn */
    public enum ETHERRCW
    {
        Specific, Forever
    }
}
