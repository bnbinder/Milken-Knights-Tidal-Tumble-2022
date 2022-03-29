// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Autonomous.Storage.EtherAuto;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERAUTO;
import frc.robot.Autonomous.Storage.EtherAuto.ETHERRCW;
import frc.robot.ToolShed.MathFormulas;

public class EtherAutoCommand extends CommandBase {
  /** Creates a new EtherAuto. */
  private double totalDistance;
  private double thetaTurn;
  private double RCWauto;
  private ETHERAUTO mode;
  private ETHERRCW turny;
  private double turnyAuto;
  
  public EtherAutoCommand(double distanceA, double lengthB, double RCWauto, double turnyAuto, ETHERAUTO mode, ETHERRCW turny)
  {
    this.RCWauto = RCWauto;
    this.mode = mode;
    this.turny = turny;
    this.totalDistance = MathFormulas.calculateArcOfPath(distanceA, lengthB);
    this.thetaTurn = MathFormulas.calculateAngleOfPath(distanceA, lengthB);
    this.turnyAuto = turnyAuto;

    //want theta turn, want turny auto, dont want rcw, curve and specific
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    EtherAuto.getInstance().etherAutoUpdate(totalDistance, thetaTurn, RCWauto, mode, turny, turnyAuto);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished();
  }
}
