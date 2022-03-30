// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.ToolShed.FalconAlgorithims;

/** Add your docs here. */
public final class Constants {

    public static final double kPi = 3.14159265359;

    public static class MKFALCON 
    {
        public static final int velocityMeasAmount = 16;
        public static final int statusOneMeas = 25;
        public static final int statusTwoMeas = 25;
        public static final double voltComp = 12;
        public static final double oneEncoderRotation = 2048;
    }

    public static class MKDRIVE 
    {
        public static final double kS = 0;
        public static final double kA = 0;
        public static final double kV = 0;
        
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final double[] pidf = {kP, kI, kD, kF};

        public static final NeutralMode mode = NeutralMode.Brake;

        public static final boolean inverted = false;

        public static final int scurve = 6;

        public static final double greerRatio = 0;

        public static final double wheelDiameterInches = 4; 
        public static final double wheelCircumference = wheelDiameterInches * kPi;    
    }

    public static class MKTURN 
    {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        
        public static final double[] pidf = {kP, kI, kD, kF};

        public static final NeutralMode mode = NeutralMode.Brake;

        public static final boolean inverted = true;

        public static final int scurve = 6;

        public static final double greerRatio = 0;
    }

    public static class MKCANCODER
    {
        public static final double topLeftOffset = -72.685546875;
        public static final double topRightOffset = -9.4921875;
        public static final double bottomLeftOffset = -117.24609375;
        public static final double bottomRightOffset = 46.0546875;

        public static final double[] offset = {MKCANCODER.topLeftOffset, MKCANCODER.topRightOffset, MKCANCODER.bottomLeftOffset, MKCANCODER.bottomRightOffset};

        public static final AbsoluteSensorRange range = AbsoluteSensorRange.Signed_PlusMinus180;

        public static final boolean inverted = true;
    }

    public static class MKTRAIN 
    {
        public static final double L = 22.57;
        public static final double W = 22.57;
        public static final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

        public static final double hP = 0.001, hI = 0.0001, hD = hP * 0.1;
    }

    public static class NAVX 
    {
        public static final double offset = 0;
    }

    public static class CONTROLLERS 
    {
        public static final int driverPort = 0;
        public static final int opPort = 1;

        public static class DRIVER 
        {
            public static final int fwd = 1;
            public static final int str = 0;
            public static final int rcw = 5;
        }

        public static final int topPOV = 0;
        public static final int rightPOV = 90;
        public static final int bottomPOV = 180;
        public static final int leftPOV = 270;
    }

    public static class CANID 
    {
        public static final int topDriveLeftCANID = 3; 
        public static final int topDriveRightCANID = 5; 
        public static final int bottomDriveLeftCANID = 2; 
        public static final int bottomDriveRightCANID = 7;

        public static final int topTurnLeftCANID = 4; 
        public static final int topTurnRightCANID = 6; 
        public static final int bottomTurnLeftCANID = 1;
        public static final int bottomTurnRightCANID = 8; 

        public static final int topTurnLeftCANCoderCANID = 16; 
        public static final int topTurnRightCANCoderCANID = 18; 
        public static final int bottomTurnLeftCANCoderCANID = 15; 
        public static final int bottomTurnRightCANCoderCANID = 17;
        
        public static final int[][] MkTrainIds =
        {
            {topDriveLeftCANID, topTurnLeftCANID, topTurnLeftCANCoderCANID},
            {topDriveRightCANID, topTurnRightCANID, topTurnRightCANCoderCANID},
            {bottomDriveLeftCANID, bottomTurnLeftCANID, bottomTurnLeftCANCoderCANID},
            {bottomDriveRightCANID, bottomTurnRightCANID, bottomTurnRightCANCoderCANID}
        };
    }

    public static class AUTO
    {
        //auto controlling pid
        public static double turnSwerveControlKp = 1;
        public static double driveSwerveControlKpY = 1;
        public static double driveSwerveControlKpX = 1;

        public static double heightMeters = FalconAlgorithims.inchesToMeters(MKTRAIN.L / 2);
        public static double widthMeters = FalconAlgorithims.inchesToMeters(MKTRAIN.W / 2);

        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
        new Translation2d(heightMeters, widthMeters),
        new Translation2d(heightMeters, -widthMeters),
        new Translation2d(-heightMeters, widthMeters),
        new Translation2d(-heightMeters, -widthMeters));
      
        
        //actual drive module stats
        public static final double maxModuleTurnVelo = kPi;
        public static final double maxModuleTurnAccel = kPi;
        
        //actual drive module stats
        public static final double maxModuleDriveVelo = 1;
        public static final double maxModuleDriveAccel = 1;
        

        //for turning constraints
        public static final double maxAutoTurnVelo = kPi;
        public static final double maxAutoTurnAccel = kPi;
        
        //for trajectory config
        public static final double maxAutoDriveVelo = 1; //2;
        public static final double maxAutoDriveAccel = 1; //2;


        public static final double maxDriveVelo = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAutoTurnVelo, maxAutoTurnAccel);
    }

}
