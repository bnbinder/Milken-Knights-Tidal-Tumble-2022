// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Commandments.Swerve;
import frc.robot.Autonomous.Commandments.Swrv;
import frc.robot.Constants.MKTURN;
import frc.robot.Constants.AUTO.DISTANGLE;
import frc.robot.Dashboard.Shuffle;
import frc.robot.Mechanisims.Climber;
import frc.robot.Mechanisims.Intake;
import frc.robot.Mechanisims.MkSwerveTrain;
import frc.robot.wpi.Odometry;

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
  //private CommandArray arr = new CommandArray("hello");
 // private RobotContainer mRobotContainer;
  private Command m_autonomousCommand;
  private double[] driverInputValues;
  private double pov;
  private Shuffle shuffle = Shuffle.getInstance();
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  @Override
  public void robotInit() {
    shuffle.startAuto();
    shuffle.startWidgets();
    Climber.getInstance().startClimb();
    Intake.getInstance().startIntake();
    train.startTrain();
    Odometry.getInstance().resetPose();
    //mRobotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    shuffle.update();
  }

  @Override
  public void autonomousInit() {
    //SmartDashboard.putNumber("af", (DISTANGLE.angleuno) % 90);
    //SmartDashboard.putNumber("pre", -((DISTANGLE.angleuno) % 90));
    //arr.addDeadline(new Turn(-((MathFormulas.calculateAngleOfPath(24, 24)) % 90)));
    //arr.addDeadline(new EtherAutoCommand(24, 24, 0, 90, ETHERAUTO.Curve, ETHERRCW.Specific));
    
    //m_autonomousCommand = AutoDriveChoose.getInstance().getSelected();
    m_autonomousCommand = new Swrv();

    if (m_autonomousCommand != null) {
     // SmartDashboard.putBoolean("yuy", true);
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    train.updateSwerve();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    train.updateSwerve();
    //Input.getInstance().mechanisims();
    Odometry.getInstance().updateOdo();
    shuffle.updateValues();
    //DriveSubsystem.getInstance().drive(driverInputValues[0], driverInputValues[1], xbox.getPOV() == 0 ? driverInputValues[2] : SwerveAlgorithims.getInstance().headerStraighter(xbox.getPOV()), true);
  }

  @Override
  public void disabledInit() {
    //arr.removeAllCommands();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    Climber.getInstance().startClimb();
    Intake.getInstance().startIntake();
    train.startTrain();
    Odometry.getInstance().resetPose();
    }

  @Override
  public void testPeriodic() {}
}
