// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Dashboard.Shuffle;
import frc.robot.Mechanisims.Climber;
import frc.robot.Mechanisims.Intake;
import frc.robot.Mechanisims.MkSwerveTrain;

/** Add your docs here. */
public class HQ 
{
    public static HQ getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void startMechanisms()
    {
        Climber.getInstance().startClimb();
        Intake.getInstance().startIntake();
        MkSwerveTrain.getInstance().startTrain();
        Shuffle.getInstance().startAuto();
        Shuffle.getInstance().startWidgets();
    }

    private static class InstanceHolder
    {
        private static final HQ mInstance = new HQ();
    } 
}
