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
  private double toggleAxisThreshold;

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

  public MkXboxInput(MkXbox joystick, int port, String name, Type type, boolean toggle, double toggleAxisThreshold)
  {
    this.joystick = joystick;
    this.port = port;
    this.name = name;
    this.type = type;
    this.toggle = toggle;
    this.isToggleOn = false;
    this.isPressed = false;
    this.toggleAxisThreshold = toggleAxisThreshold;
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
      if(toggle && type == Type.Button && joystick.getRawButton(port))
      {
          
            if(!isPressed)
            {
                isToggleOn = !isToggleOn;
                isPressed = true;
            }
            else
            {
              isPressed = false;
            }
          
      }
      else if(toggle && type == Type.Axis && Math.abs(joystick.getRawAxis(port)) > toggleAxisThreshold)
      {
          if(!isPressed)
          {
              isToggleOn = !isToggleOn;
              isPressed = true;
          }
          else
          {
            isPressed = false;
          }
        
      }
      else
      {
        isToggleOn = false;
      }
      return isToggleOn;
  }

  public double getAxis()
  {
    if(type == Type.Axis)
    {
      return joystick.getRawAxis(port);
    }
    else
    {
      return 0;
    }
  }

  public enum Type
  {
      Axis, Button;
  }
}
