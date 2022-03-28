// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class MkXbox extends XboxController{
    private HashMap<Integer, MkXboxInput> inputs;

    /**
     * Create a new MkJoystick.
     */
    public MkXbox(int port) 
    {
      super(port);
      inputs = new HashMap<Integer, MkXboxInput>();
    }
  
    /**
     * Gets a button of the joystick. Creates a new Button object if one did not already exist.
     *
     * @param button The raw button number of the button to get
     * @return The button
     */
    public MkXboxInput getButton(int port, String name) {
      if (!inputs.containsKey(port)) {
        inputs.put(port, new MkXboxInput(this, port, name));
      }
      return inputs.get(port);
    }
  }
}
