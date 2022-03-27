// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Add your docs here. */
public final class Constants {

    public static final double kPi = 3.14159265359;

    public static class MKFALCON 
    {
        public static final int velocityMeasAmount = 16;
        public static final int statusOneMeas = 25;
        public static final int statusTwoMeas = 25;
        public static final double voltComp = 12;
        public static double oneEncoderRotation = 2048;
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

    public static class MKTRAIN 
    {
        public static final double[][][] pidf = 
        {
            {
                MKDRIVE.pidf,
                MKTURN.pidf
            },
            {
                MKDRIVE.pidf,
                MKTURN.pidf
            },
            {
                MKDRIVE.pidf,
                MKTURN.pidf
            },
            {
                MKDRIVE.pidf,
                MKTURN.pidf
            },
        };

        public static final NeutralMode[][] mode = 
        {
            {MKDRIVE.mode, MKTURN.mode},
            {MKDRIVE.mode, MKTURN.mode},
            {MKDRIVE.mode, MKTURN.mode},
            {MKDRIVE.mode, MKTURN.mode}
        };

        public static final boolean[][] inverted =
        {
            {MKDRIVE.inverted, MKTURN.inverted},
            {MKDRIVE.inverted, MKTURN.inverted},
            {MKDRIVE.inverted, MKTURN.inverted},
            {MKDRIVE.inverted, MKTURN.inverted}
        };

        public static final int[][] scurve = 
        {
            {MKDRIVE.scurve, MKTURN.scurve},
            {MKDRIVE.scurve, MKTURN.scurve},
            {MKDRIVE.scurve, MKTURN.scurve},
            {MKDRIVE.scurve, MKTURN.scurve}
        };

        public static final double L = 22.57;
        public static final double W = 22.57;
        public static final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    }

    public static class NAVX 
    {
        public static final double offset = 0;
    }

    public static class CANID 
    {
        public static int topDriveLeftCANID = 3; 
        public static int topDriveRightCANID = 5; 
        public static int bottomDriveLeftCANID = 2; 
        public static int bottomDriveRightCANID = 7;

        public static int topTurnLeftCANID = 4; 
        public static int topTurnRightCANID = 6; 
        public static int bottomTurnLeftCANID = 1;
        public static int bottomTurnRightCANID = 8; 
        
        public static int[][] MkTrainIds =
        {
            {topDriveLeftCANID, topTurnLeftCANID},
            {topDriveRightCANID, topTurnRightCANID},
            {bottomDriveLeftCANID, bottomTurnLeftCANID},
            {bottomDriveRightCANID, bottomTurnRightCANID}
        };
    }
}
