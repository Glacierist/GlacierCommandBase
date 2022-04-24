// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /* - - - SWERVE DRIVE CONSTANTS - - - */
    public static final int frontLeftDriveID = 4;
    public static final int frontRightDriveID = 2;
    public static final int backLeftDriveID = 6;
    public static final int backRightDriveID = 8;

    public static final int frontLeftTurnID = 3;
    public static final int frontRightTurnID = 1;
    public static final int backLeftTurnID = 5;
    public static final int backRightTurnID = 7;

    public static final int frontLeftEncoderPort = 1;
    public static final int frontRightEncoderPort = 0;
    public static final int backLeftEncoderPort = 2;
    public static final int backRightEncoderPort = 3;

    public static final double driveEncoderVelocityConversion = (1/5.3333) * (0.106 * 3.14) * (1/60);
    public static final double turnEncoderPositionConversion = 360 * (1/11.6571);

    public static final ProfiledPIDController turnPIDController = new ProfiledPIDController(0.00, 0.00, 0.00, new TrapezoidProfile.Constraints(5, 8));
    public static final ProfiledPIDController drivePIDController = new ProfiledPIDController(0.00, 0.00, 0.00, new TrapezoidProfile.Constraints(10, 20));
    public static final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.55493, 2.3014, 0.51488);

    public static final double drivetrainModuleOffset = 0.2923;
    public static final int numberOfModules = 4;
    public static final double maxVelocityMultiplier = 4;
    public static final double radiansPerSecondMultiplier = 6;

    /* - - - OTHER CONSTANTS - - - */
    public static final int swerveControllerPort = 0;
    public static final int alternateControllerPort = 1;

    /* - - - SHOOTER CONSTANTS - - - */
    public static final int hoodMotorID = 11;
    public static final int flywheelLeftID = 4;
    public static final int flywheelRightID = 5;

    /* - - - INDEXER CONSTANTS - - - */
    public static final int outerIndexerID = 15;
    public static final int innerIndexerID = 12;
    public static final double outerIndexerSpeed = 0;
    public static final double innerIndexerSpeed = 0;
    
    /* - - - INTAKE CONSTANTS - - - */
    public static final int intakeMotorID = 16;
    public static final int pneumaticsHubID = 0;
    public static final int solenoidOne = 14;
    public static final int solenoidTwo = 15;
    public static final double intakeSpeed = 0.4;

    /* - - - CLIMBER CONSTANTS - - - */
    public static final int rightClimberID = 9;
    public static final int leftClimberID = 13;
    public static final double climberSpeed = 0.4;

    /* - - - LIMELIGHT CONSTANTS - - - */
    public static final double limelightHeight = 0.5 /*meters*/;
    public static final double hubHeight = 2.64 /*meters*/;
    public static final double limelightAngle = 25 /*degrees*/;
}
