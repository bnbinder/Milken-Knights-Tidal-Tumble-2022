// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wpi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.navx;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Factory.Motor.MkSwerveModule;
import frc.robot.Factory.Motor.MkSwerveTrain;
import frc.robot.ToolShed.FalconAlgorithims;

/** Add your docs here. */
public class Odometry {

   /* private TalonFX topDriveLeft = new TalonFX(DRIVE.topDriveLeftCANID);
    private TalonFX topDriveRight = new TalonFX(DRIVE.topDriveRightCANID);
    private TalonFX bottomDriveLeft = new TalonFX(DRIVE.bottomDriveLeftCANID);
    private TalonFX bottomDriveRight = new TalonFX(DRIVE.bottomDriveRightCANID);
*/

    // Locations for the swerve drive modules relative to the robot center.

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = AUTO.kDriveKinematics;

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    private final MkSwerveModule m_frontLeft = MkSwerveTrain.getInstance().getModules()[0];//new SwerveModule(DRIVE.topDriveLeftCANID, TURN.topTurnLeftCANID, TURN.topTurnLeftCANCoderCANID);
    private final MkSwerveModule m_frontRight = MkSwerveTrain.getInstance().getModules()[1]; //new SwerveModule(DRIVE.topDriveRightCANID, TURN.topTurnRightCANID, TURN.topTurnRightCANCoderCANID);
    private final MkSwerveModule m_backLeft = MkSwerveTrain.getInstance().getModules()[2]; //new SwerveModule(DRIVE.bottomDriveLeftCANID, TURN.bottomTurnLeftCANID, TURN.bottomTurnLeftCANCoderCANID);
    private final MkSwerveModule m_backRight = MkSwerveTrain.getInstance().getModules()[3];// new SwerveModule(DRIVE.bottomDriveRightCANID, TURN.bottomTurnRightCANID, TURN.bottomTurnRightCANCoderCANID);

    
    Rotation2d navX2d = new Rotation2d(Math.toRadians(navx.getInstance().getNavxYaw()));
    
    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
    navX2d, new Pose2d(0, 0, new Rotation2d()));

    Pose2d m_pose = new Pose2d();

    public Odometry()
    {

    }

    public static Odometry getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateOdo()
    {
        navX2d = Rotation2d.fromDegrees(navx.getInstance().getNavxYaw());
            // Update the pose
            
        m_pose = m_odometry.update(
            navX2d,
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navX2d)
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, FalconAlgorithims.nativePer100MsToMetersPerSec(MKDRIVE.maxNativeVelocity));
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public double getX()
    {
        return m_pose.getX();
    }

    public double getY()
    {
        return m_pose.getY();
    }

    public void resetPose()
    {
        m_odometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), new Rotation2d());
    }

    public Pose2d getPose()
    {
        return m_pose;
    }

    private static class InstanceHolder
    {
        private static final Odometry mInstance = new Odometry();
    } 
    
}