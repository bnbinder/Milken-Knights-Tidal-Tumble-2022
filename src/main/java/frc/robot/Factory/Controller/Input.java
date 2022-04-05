// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factory.Controller;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CONTROLLERS.ShootInput;
import frc.robot.Constants.CONTROLLERS.ClimbInput;
import frc.robot.Constants.CONTROLLERS.DriveInput;
import frc.robot.Constants.CONTROLLERS.ElevatorInput;
import frc.robot.Constants.CONTROLLERS.IntakeInput;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.CONTROLLERS;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Constants.SHOOTER;
import frc.robot.Factory.Controller.MkXboxInput.Type;
import frc.robot.Mechanisims.Climber;
import frc.robot.Mechanisims.Elevator;
import frc.robot.Mechanisims.Intake;
import frc.robot.Mechanisims.MkSwerveTrain;
import frc.robot.Mechanisims.Shooter;
import frc.robot.ToolShed.SwerveAlgorithims;
import frc.robot.wpi.Odometry;

/** Add your docs here. */
public class Input {
    private XboxController xbox = new XboxController(0);
    private XboxController op = new XboxController(1);
    private MkXboxInput fakeLimelightInput = new MkXboxInput(xbox, CONTROLLERS.fakeLimelight, Type.Button, false);
    private MkXboxInput[] driveInput = {new MkXboxInput(xbox, DriveInput.fwd, Type.Axis, false, 0.1), new MkXboxInput(xbox, DriveInput.str, Type.Axis, false, 0.1), new MkXboxInput(xbox, DriveInput.rcw, Type.Axis, false, 0.1)};
    private MkXboxInput intakeInput = new MkXboxInput(op, IntakeInput.intakeButton, Type.Button, true);
    private MkXboxInput rollerInput[] = {new MkXboxInput(op, IntakeInput.rollerForwardBumper, Type.Axis, false, 0.1), new MkXboxInput(op, IntakeInput.rollerBackwardBumper, Type.Axis, false, 0.1)};
    private MkXboxInput[] shooterInput = {new MkXboxInput(op, ShootInput.forwardShootTrigger, Type.Axis, false, 0.1), new MkXboxInput(op, ShootInput.backwardShootTrigger, Type.Axis, false, 0.1)};
    private MkXboxInput climbInput = new MkXboxInput(op, ClimbInput.climbAxis, Type.Axis, false);
    private MkXboxInput autoClimbInput = new MkXboxInput(op, ClimbInput.autoClimbButton, Type.Button, true);
    private MkXboxInput elevatorInput = new MkXboxInput(op, ElevatorInput.elevatorAxis, Type.Axis, false, 0.1);
    private double pov = xbox.getPOV();
    private SwerveAlgorithims swerve = SwerveAlgorithims.getInstance();
    private Shooter shoot = Shooter.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Climber climb = Climber.getInstance();
    private Intake intake = Intake.getInstance();
    private double fwd, str, rcw;

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

        if(rollerInput[0].isPressed())
        {
            intake.setRoller(INTAKE.rollerPercentSpeed, ControlMode.PercentOutput);
        }
        else if(rollerInput[1].isPressed())
        {
            intake.setRoller(-INTAKE.rollerPercentSpeed, ControlMode.PercentOutput);
        }
        else  
        {
            intake.setRoller(0, ControlMode.PercentOutput);
        }
    }

    public void climb()
    {/*
        if(pov == ClimbInput.leftClimbUpPOV)
        {
            climb.setLeftClimb(CLIMBER.climbUpSpeed, ControlMode.PercentOutput);
        }
        else if(pov == ClimbInput.leftClimbDownPOV)
        {
            climb.setLeftClimb(CLIMBER.climbDownSpeed, ControlMode.PercentOutput);
        }

        if(pov == ClimbInput.rightClimbUpPOV)
        {
            climb.setRightClimb(CLIMBER.climbUpSpeed, ControlMode.PercentOutput);
        }
        else if(pov == ClimbInput.rightClimbDownPOV)
        {
            climb.setRightClimb(CLIMBER.climbDownSpeed, ControlMode.PercentOutput);
        }

        if(Math.abs(climbInput.getAxis()) > 0.1)
        {
            climb.setClimb(Math.abs(climbInput.getAxis()) * (climbInput.getAxis() < 0.1 ? CLIMBER.climbDownSpeed : CLIMBER.climbUpSpeed), ControlMode.PercentOutput);
        }

        if(pov != ClimbInput.leftClimbUpPOV && pov != ClimbInput.leftClimbDownPOV && pov != ClimbInput.rightClimbUpPOV && pov != ClimbInput.rightClimbDownPOV && Math.abs(climbInput.getAxis()) < 0.1)
        {
            climb.setClimb(0, ControlMode.PercentOutput);
        }*/
        SmartDashboard.putNumber("pov", pov);
    }

    public void drive()
    {
        fwd = driveInput[0].getAxis();
        str = -driveInput[1].getAxis();
        rcw = (pov == -1) ? (fakeLimelightInput.isPressed() ? swerve.headerStraighter(Odometry.getInstance().getAngleFromGoal()) : driveInput[2].getAxis()) : swerve.headerStraighter(pov);
        if(fwd != 0 || str != 0 || rcw != 0)
        {
            //MkSwerveTrain.getInstance().getModules()[0].turnMotor().setFalcon(ControlMode.Velocity, fwd * 90);
            swerve.etherSwerve(fwd / MKTRAIN.speedLimit,str / MKTRAIN.speedLimit, rcw / MKTRAIN.speedLimit);
        }
        else
        {
            MkSwerveTrain.getInstance().stopEverything();
        }
        //swerve.etherSwerve(0, 0, 0);
        //SmartDashboard.putNumber("axis", driveInput[1].getAxis());
    }

    public void mechanisims()
    {
        drive();
        //intake();
        //elevator();
        //shooter();
        //climb();
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
