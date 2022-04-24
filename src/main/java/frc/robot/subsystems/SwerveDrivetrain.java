// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;
  
  private Translation2d frontLeftModuleLocation;
  private Translation2d frontRightModuleLocation;
  private Translation2d backLeftModuleLocation;
  private Translation2d backRightModuleLocation;

  private ChassisSpeeds drivetrainSpeeds;
  private SwerveModuleState[] moduleStates;
  private SwerveDriveKinematics moduleDistances;

  private AHRS gyro;

  public SwerveDrivetrain() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    /* Let the code know where the modules are in relation to the origin of the robot */
    frontLeftModuleLocation = new Translation2d(Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    frontRightModuleLocation = new Translation2d(-Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    backLeftModuleLocation = new Translation2d(Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);
    backRightModuleLocation = new Translation2d(-Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);

    /* Creates one swerve module state for each module (4) */
    /* The array order (of moduleStates) corresponds to the order of modules in moduleDistances */
    /* This is also the same order you recieve module states from inverse kinematics */
    moduleDistances = new SwerveDriveKinematics(frontLeftModuleLocation, frontRightModuleLocation, backLeftModuleLocation, backRightModuleLocation);
    moduleStates = new SwerveModuleState[Constants.numberOfModules];


    /* Create swerve module objects using the SwerveModule subsystem */
    frontLeftModule = new SwerveModule(Constants.frontLeftTurnID, Constants.frontLeftDriveID, Constants.frontLeftEncoderPort);
    frontRightModule = new SwerveModule(1, 2, 0);
    backLeftModule = new SwerveModule(5, 6, 2);
    backRightModule = new SwerveModule(7, 8, 3);

    /* Create a gyro */
    try {
      gyro = new AHRS(SPI.Port.kMXP); 
      gyro.reset();
    } catch (RuntimeException ex) {
        System.out.println("--------------");
        System.out.println("NavX not plugged in");
        System.out.println("--------------");
    } 
  }

  /* Converts forward, strafe, and yaw inputs (all from -1 to 1) to module angles and drive velocities */
  /* Also includes angle optimization, for example: turning the module -90 degrees instead of going 270 degrees */
  public void periodicModuleUpdate(double forward, double strafe, double yaw) {
    drivetrainSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds
    (strafe, forward, yaw, Rotation2d.fromDegrees(-gyro.getAngle()));

    moduleStates = moduleDistances.toSwerveModuleStates(drivetrainSpeeds);

    /* With module angle optimization */
    var frontLeftOptimized = SwerveModuleState.optimize(moduleStates[0], frontLeftModule.getCurrentAngle());
    var frontRightOptimized = SwerveModuleState.optimize(moduleStates[1], frontRightModule.getCurrentAngle());
    var backLeftOptimized = SwerveModuleState.optimize(moduleStates[2], backLeftModule.getCurrentAngle());
    var backRightOptimized = SwerveModuleState.optimize(moduleStates[3], backRightModule.getCurrentAngle());
    
    frontLeftModule.setModule(frontLeftOptimized.angle, frontLeftOptimized.speedMetersPerSecond);
    frontRightModule.setModule(frontRightOptimized.angle, frontRightOptimized.speedMetersPerSecond);
    backLeftModule.setModule(backLeftOptimized.angle, backLeftOptimized.speedMetersPerSecond);
    backRightModule.setModule(backRightOptimized.angle, backRightOptimized.speedMetersPerSecond);

    /* No module angle optimization */
    // frontLeftModule.setModule(moduleStates[0].angle, moduleStates[0].speedMetersPerSecond);
    // frontRightModule.setModule(moduleStates[1].angle, moduleStates[1].speedMetersPerSecond);
    // backLeftModule.setModule(moduleStates[2].angle, moduleStates[2].speedMetersPerSecond);
    // backRightModule.setModule(moduleStates[3].angle, moduleStates[3].speedMetersPerSecond);
  }
}
