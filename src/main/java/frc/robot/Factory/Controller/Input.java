// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.CONTROLLERS.DRIVER;
import frc.robot.Factory.Controller.MkXboxInput.Type;
import frc.robot.ToolShed.SwerveAlgorithims;

/** Add your docs here. */
public class Input {
    private XboxController xbox = new XboxController(0);
    private MkXboxInput[] driveInput = {new MkXboxInput(xbox, DRIVER.fwd, Type.Axis, false, 0.1), new MkXboxInput(xbox, DRIVER.str, Type.Axis, false, 0.1), new MkXboxInput(xbox, DRIVER.rcw, Type.Axis, false, 0.1)};
    private double pov = xbox.getPOV();

    public static Input getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public double[] getDriveInput()
    {
        return new double[] {driveInput[0].getAxis(), -driveInput[1].getAxis(), driveInput[2].getAxis()};
    }

    public void drive()
    {
        SwerveAlgorithims.getInstance().etherSwerve(driveInput[0].getAxis(), -driveInput[1].getAxis(), pov == 0 ? driveInput[2].getAxis() : SwerveAlgorithims.getInstance().headerStraighter(pov));
    }

    public double getPOV()
    {
        return xbox.getPOV();
    }

    private static class InstanceHolder
    {
        private static final Input mInstance = new Input();
    } 
}
