// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Autonomous.Storage.TurnAuto;
import frc.robot.Factory.MkSwerveTrain;

public class Turn extends CommandBase {
  /** Creates a new Turn. */
  private double angle;
  public Turn(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MkSwerveTrain.getInstance().setModuleTurn(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TurnAuto.getInstance().isFinished();
  }
}
