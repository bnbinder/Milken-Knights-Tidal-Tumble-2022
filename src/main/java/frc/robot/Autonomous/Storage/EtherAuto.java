// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Storage;

import frc.robot.Constants;
import frc.robot.Factory.MkSwerveTrain;
import frc.robot.ToolShed.Ether;

/** Add your docs here. */
public class EtherAuto {
    private double STRauto;
    private double FWDauto;
    private double totalDistance;
    private double avgDistInches;

    private EtherAuto()
    {
        
    }

    public static EtherAuto getInstance()
    {
        return InstanceHolder.mInstance;
    }

    //turn distance is degrees

    public void setEtherAuto(double totalDistance)
    {
        this.totalDistance = totalDistance;
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
    public void etherAutoUpdate(double totalDistance, double thetaTurn, double RCWauto, ETHERAUTO mode, ETHERRCW turny, double turnyAuto)
    {
        double RCWtemp = RCWauto;
        avgDistInches = MkSwerveTrain.getInstance().vars.avgDistInches;
        if(mode == ETHERAUTO.Curve)
        {
            FWDauto = Math.cos((-1 * thetaTurn) + (2 * ((avgDistInches/totalDistance)*thetaTurn)) * Constants.kPi / 180);
            STRauto = Math.sin((-1 * thetaTurn) + (2 * ((avgDistInches/totalDistance)*thetaTurn)) * Constants.kPi / 180);
        }
        else if(mode == ETHERAUTO.Straight)
        {
            FWDauto = Math.cos(thetaTurn);
            STRauto = Math.sin(thetaTurn);
        }
        if(turny == ETHERRCW.Specific)
        {
            RCWtemp = Ether.getInstance().headerStraighter(turnyAuto);
        }
        Ether.getInstance().etherSwerve(-FWDauto, STRauto, RCWtemp);
    }

    public boolean isFinished()
    {
        return avgDistInches >= Math.abs(totalDistance) - 1;
    }

    private static class InstanceHolder
    {
        private static final EtherAuto mInstance = new EtherAuto();
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
