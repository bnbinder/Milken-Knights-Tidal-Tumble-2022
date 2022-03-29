// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class MkXboxInput {
  private XboxController joystick;
  private int port;
  private boolean isToggleOn;
  private boolean isTogglePressed;
  private boolean isPressed;
  private Type type;
  private boolean toggle;
  private double toggleAxisThreshold;

  public MkXboxInput(XboxController joystick, int port, Type type, boolean toggle)
  {
    this.joystick = joystick;
    this.port = port;
    this.type = type;
    this.toggle = toggle;
    this.isToggleOn = false;
    this.isPressed = false;
  }

  public MkXboxInput(XboxController joystick, int port, Type type, boolean toggle, double toggleAxisThreshold)
  {
    this.joystick = joystick;
    this.port = port;
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
          
            if(!isTogglePressed)
            {
                isToggleOn = !isToggleOn;
                isTogglePressed = true;
            }
            else
            {
              isTogglePressed = false;
            }
      }
      else if(toggle && type == Type.Axis && Math.abs(joystick.getRawAxis(port)) > toggleAxisThreshold)
      {
          if(!isTogglePressed)
          {
              isToggleOn = !isToggleOn;
              isTogglePressed = true;
          }
          else
          {
            isTogglePressed = false;
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

  public int getPort()
  {
    return port;
  }

  public enum Type
  {
      Axis, Button;
  }
}
