// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** Add your docs here. */
public class AutoDriveChoose {
    private SendableChooser<AutoPosition> chooser = new SendableChooser<>();
    private ShuffleboardTab mTab = Shuffleboard.getTab("Match");

    private AutoDriveChoose()
    {
        Shuffleboard.selectTab("Match");
        for(AutoPosition i : AutoPosition.values())
        {
            chooser.addOption(i.toString(), i);
        } 
        chooser.setDefaultOption("LEFT", AutoPosition.LEFT);
    }

    public static AutoDriveChoose getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public AutoPosition getSelected()
    {
        return chooser.getSelected();
    }

    public AutoPosition[] getValues()
    {
        return AutoPosition.values();
    }

    private static class InstanceHolder
    {
        private static final AutoDriveChoose mInstance = new AutoDriveChoose();
    } 

    public enum AutoPosition
    {
        LEFT, NOTHING
    }
}
