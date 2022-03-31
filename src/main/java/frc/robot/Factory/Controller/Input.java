// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.CONTROLLERS.ShootInput;
import frc.robot.Constants.CONTROLLERS.ClimbInput;
import frc.robot.Constants.CONTROLLERS.DriveInput;
import frc.robot.Constants.CONTROLLERS.ElevatorInput;
import frc.robot.Constants.CONTROLLERS.IntakeInput;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.SHOOTER;
import frc.robot.Factory.Controller.MkXboxInput.Type;
import frc.robot.Mechanisims.Climber;
import frc.robot.Mechanisims.Elevator;
import frc.robot.Mechanisims.Intake;
import frc.robot.Mechanisims.Shooter;
import frc.robot.ToolShed.SwerveAlgorithims;

/** Add your docs here. */
public class Input {
    private XboxController xbox = new XboxController(0);
    private MkXboxInput[] driveInput = {new MkXboxInput(xbox, DriveInput.fwd, Type.Axis, false, 0.1), new MkXboxInput(xbox, DriveInput.str, Type.Axis, false, 0.1), new MkXboxInput(xbox, DriveInput.rcw, Type.Axis, false, 0.1)};
    private MkXboxInput intakeInput = new MkXboxInput(xbox, IntakeInput.intake, Type.Button, true);
    private MkXboxInput rollerInput = new MkXboxInput(xbox, IntakeInput.roller, Type.Axis, false, 0.1);
    private MkXboxInput[] shooterInput = {new MkXboxInput(xbox, ShootInput.forwardShoot, Type.Axis, false, 0.1), new MkXboxInput(xbox, ShootInput.backwardShoot, Type.Axis, false, 0.1)};
    private MkXboxInput[] climberLeftInput = {new MkXboxInput(xbox, ClimbInput.leftClimbUp, Type.Button, false), new MkXboxInput(xbox, ClimbInput.leftClimbDown, Type.Button, false)};
    private MkXboxInput[] climberRightInput = {new MkXboxInput(xbox, ClimbInput.rightClimbUp, Type.Button, false), new MkXboxInput(xbox, ClimbInput.rightClimbDown, Type.Button, false)};
    private MkXboxInput climbInput = new MkXboxInput(xbox, ClimbInput.climb, Type.Axis, false);
    private MkXboxInput autoClimbInput = new MkXboxInput(xbox, ClimbInput.autoClimb, Type.Button, true);
    private MkXboxInput elevatorInput = new MkXboxInput(xbox, ElevatorInput.elevator, Type.Axis, false, 0.1);
    private double pov = xbox.getPOV();
    private SwerveAlgorithims swerve = SwerveAlgorithims.getInstance();
    private Shooter shoot = Shooter.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Climber climb = Climber.getInstance();
    private Intake intake = Intake.getInstance();

    public static Input getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public double[] getDriveInput()
    {
        return new double[] {driveInput[0].getAxis(), -driveInput[1].getAxis(), driveInput[2].getAxis()};
    }

    public void elevator()
    {
        if(Math.abs(elevatorInput.getAxis()) > 0)
        {
            elevator.setElevator(elevatorInput.getAxis() * (elevatorInput.getAxis() < 0 ? ELEVATOR.elevatorPercentSpeedBack : ELEVATOR.elevatorPercentSpeedForward), ControlMode.PercentOutput);
        }
        else 
        {
            elevator.setElevator(0, ControlMode.PercentOutput);
        }
    }

    public void shooter()
    {
        if(Math.abs(shooterInput[0].getAxis()) > 0)
        {
            shoot.setLeftShoot(-SHOOTER.maxNativeShooterVelocity * shooterInput[0].getAxis(), ControlMode.Velocity);
        }
        else if(Math.abs(shooterInput[1].getAxis()) > 0)
        {
            shoot.setRightShoot(SHOOTER.maxNativeShooterVelocity * shooterInput[1].getAxis(), ControlMode.Velocity);
        }
        else
        {
            shoot.setShoot(0, ControlMode.PercentOutput);
        } 
    }

    public void intake()
    {
        if(intakeInput.isToggled())
        {
            intake.setIntake(INTAKE.maxIntakeNativePosition, ControlMode.Position);
        }
        else
        {
            intake.setIntake(0, ControlMode.Position);
        }

        if(Math.abs(rollerInput.getAxis()) > 0)
        {
            intake.setRoller(rollerInput.getAxis() * INTAKE.rollerPercentSpeed, ControlMode.PercentOutput);
        }
        else  
        {
            intake.setRoller(0, ControlMode.PercentOutput);
        }
    }

    public void climb()
    {
        if(climberLeftInput[0].isPressed())
        {
            climb.setLeftClimb(CLIMBER.climbUpSpeed, ControlMode.PercentOutput);
        }
        else if(climberLeftInput[1].isPressed())
        {
            climb.setLeftClimb(CLIMBER.climbDownSpeed, ControlMode.PercentOutput);
        }

        if(climberRightInput[0].isPressed())
        {
            climb.setRightClimb(CLIMBER.climbUpSpeed, ControlMode.PercentOutput);
        }
        else if(climberRightInput[1].isPressed())
        {
            climb.setRightClimb(CLIMBER.climbDownSpeed, ControlMode.PercentOutput);
        }

        if(Math.abs(climbInput.getAxis()) > 0)
        {
            climb.setClimb(Math.abs(climbInput.getAxis()) * (climbInput.getAxis() < 0 ? CLIMBER.climbDownSpeed : CLIMBER.climbUpSpeed), ControlMode.PercentOutput);
        }

        if(!climberLeftInput[0].isPressed() && !climberLeftInput[1].isPressed() && !climberRightInput[0].isPressed() && !climberRightInput[1].isPressed() && Math.abs(climbInput.getAxis()) < climbInput.getThreshold())
        {
            climb.setClimb(0, ControlMode.PercentOutput);
        }
    }

    public void drive()
    {
        swerve.etherSwerve(driveInput[0].getAxis(), -driveInput[1].getAxis(), pov == 0 ? driveInput[2].getAxis() : swerve.headerStraighter(pov));
    }

    public void mechanisims()
    {
        drive();
        intake();
        elevator();
        shooter();
        climb();
    }

    public double getPOV()
    {
        return xbox.getPOV();
    }

    private static class InstanceHolder
    {
        private static final Input mInstance = new Input();
    } 
}
