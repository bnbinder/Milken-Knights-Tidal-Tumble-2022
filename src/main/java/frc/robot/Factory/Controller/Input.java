// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

/** Add your docs here. */
public class Input {

    public static Input getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public double[] getDriveInput(MkXboxInput... input)
    {
        return new double[] {input[0].getAxis(), input[1].getAxis(), input[2].getAxis()};
    }

    private static class InstanceHolder
    {
        private static final Input mInstance = new Input();
    } 
}
