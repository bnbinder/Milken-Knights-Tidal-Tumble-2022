// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisims;

/** Add your docs here. */
public class Shooter 
{
    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    } 
}
