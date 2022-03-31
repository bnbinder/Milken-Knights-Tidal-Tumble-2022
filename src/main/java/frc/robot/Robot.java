// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Commands.EtherAutoCommand;
import frc.robot.Autonomous.Commands.Turn;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERAUTO;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERRCW;
import frc.robot.Dashboard.Shuffle;
import frc.robot.Factory.Controller.Input;
import frc.robot.Mechanisims.MkSwerveTrain;
import frc.robot.ToolShed.CommandArray;
import frc.robot.ToolShed.MathFormulas;
import frc.robot.wpi.RobotContainer;

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
  private RobotContainer mRobotContainer;
  private Command m_autonomousCommand;
  private double[] driverInputValues;
  private double pov;
  @Override
  public void robotInit() {
    Shuffle.getInstance().startAuto();
    Shuffle.getInstance().startWidgets();
    mRobotContainer = new RobotContainer();
    MkSwerveTrain.getInstance().startTrain();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().update();
  }

  @Override
  public void autonomousInit() {
    arr.addCommand(new Turn(-((MathFormulas.calculateAngleOfPath(24, 24)) % 90)));
    arr.addCommand(new EtherAutoCommand(24, 24, 0, 90, ETHERAUTO.Curve, ETHERRCW.Specific));
    m_autonomousCommand = arr.asSequentialCommandGroup();
    //m_autonomousCommand = AutoDriveChoose.getInstance().getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Input.getInstance().drive();
    //DriveSubsystem.getInstance().drive(driverInputValues[0], driverInputValues[1], xbox.getPOV() == 0 ? driverInputValues[2] : SwerveAlgorithims.getInstance().headerStraighter(xbox.getPOV()), true);
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
