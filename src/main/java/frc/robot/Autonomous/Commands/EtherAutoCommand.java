// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Autonomous.Storage.EtherAuto;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERAUTO;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERRCW;
import frc.robot.Constants.AUTO.DISTANGLE;
import frc.robot.Mechanisims.MkSwerveTrain;
import frc.robot.ToolShed.MathFormulas;

public class EtherAutoCommand extends CommandBase {
  /** Creates a new EtherAuto. */
  private double totalDistance;
  private double thetaTurn;
  private double RCWauto;
  private ETHERAUTO mode;
  private ETHERRCW turny;
  private double turnyAuto;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private EtherAuto ether = EtherAuto.getInstance();
  
  public EtherAutoCommand(double dist, double ang, double RCWauto, double turnyAuto, ETHERAUTO mode, ETHERRCW turny)
  {
    this.RCWauto = RCWauto;
    this.mode = mode;
    this.turny = turny;
    this.totalDistance = dist;
    this.thetaTurn = ang;
    this.turnyAuto = turnyAuto;

    //want theta turn, want turny auto, dont want rcw, curve and specific
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    train.startDrive();
    train.setDist(totalDistance);
    train.setMagic();
    ether.setEtherAuto(totalDistance);
    //SmartDashboard.putBoolean("key", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ether.etherAutoUpdate(thetaTurn, RCWauto, mode, turny, turnyAuto);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //MkSwerveTrain.getInstance().stopEverything();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // SmartDashboard.putBoolean("false", false);
    return ether.isFinished(); 
  }
}
