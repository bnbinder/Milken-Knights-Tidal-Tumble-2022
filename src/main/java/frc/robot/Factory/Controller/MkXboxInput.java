// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

/** Add your docs here. */
public class MkXboxInput {
  private MkXbox joystick;
  private int port;
  private String name;
  private boolean isToggleOn;
  private boolean isTogglePressed;
  private boolean isPressed;
  private Type type;
  private boolean toggle;

  public MkXboxInput(MkXbox joystick, int port, String name, Type type, boolean toggle)
  {
    this.joystick = joystick;
    this.port = port;
    this.name = name;
    this.type = type;
    this.toggle = toggle;
    this.isToggleOn = false;
    this.isPressed = false;
  }

  public boolean isPressed()
  {
    if(type == Type.Axis)
    {
        isPressed = false;
    }
    else
    {
        if(joystick.getRawButton(port))
        {
            isPressed = true;;
        }
        else
        {
            isPressed = false;
        }
    }
    return isPressed;
  }

  public boolean isToggled()
  {
      if(toggle)
      {
          if(joystick.getRawButton(port))
          {
            if(!isPressed)
            {
                isToggleOn = !isToggleOn;
                isPressed = true;
            }
          }
      }
  }

  public enum Type
  {
      Axis, Button;
  }
}
