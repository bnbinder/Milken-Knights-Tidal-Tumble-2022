// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Commands.EtherAutoCommand;
import frc.robot.Autonomous.Commands.Turn;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERAUTO;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERRCW;
import frc.robot.Constants.CONTROLLERS.DRIVER;
import frc.robot.Factory.Controller.Input;
import frc.robot.Factory.Controller.MkXboxInput;
import frc.robot.Factory.Controller.MkXboxInput.Type;
import frc.robot.ToolShed.CommandArray;
import frc.robot.ToolShed.MathFormulas;
import frc.robot.ToolShed.SwerveAlgorithims;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private CommandArray arr = new CommandArray("hello");
  private XboxController xbox = new XboxController(0);
  private MkXboxInput[] driveInput = {new MkXboxInput(xbox, DRIVER.fwd, Type.Axis, false, 0.1), new MkXboxInput(xbox, DRIVER.str, Type.Axis, false, 0.1), new MkXboxInput(xbox, DRIVER.rcw, Type.Axis, false, 0.1)};
  private double[] driverInputValues;
  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    arr.addCommand(new Turn(-((MathFormulas.calculateAngleOfPath(24, 24)) % 90)));
    arr.addCommand(new EtherAutoCommand(24, 24, 0, 90, ETHERAUTO.Curve, ETHERRCW.Specific));
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    driverInputValues = Input.getInstance().getDriveInput(driveInput);
    SwerveAlgorithims.getInstance().etherSwerve(driverInputValues[0], driverInputValues[1], driverInputValues[2]);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
