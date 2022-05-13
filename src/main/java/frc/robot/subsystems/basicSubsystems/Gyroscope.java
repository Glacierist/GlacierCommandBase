// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Limelight;

public class Gyroscope extends SubsystemBase {
  private AHRS gyro;
  private Limelight limelight;
  private SwerveDrivetrain swerveDrive;

  /** Creates a new Gyro. */
  public Gyroscope(double offset) {

    limelight = RobotContainer.limelight;
    swerveDrive = RobotContainer.swerveDrive;
    
    /* Create a gyro */
    try {
      gyro = new AHRS(SPI.Port.kMXP); 
      gyro.reset();
    } 
    catch (RuntimeException ex) {
        System.out.println("--------------");
        System.out.println("NavX not plugged in");
        System.out.println("--------------");
    } 

    gyro.setAngleAdjustment(offset);
  }

  /* Returns the total angle (goes past 360) */
  /* ANGLE ADJUSTMENT DOES EFFECT THIS VALUE */
  public double getTotalAngle() {
    return gyro.getAngle();
  }

  /* Returns an angle from -180 to 180 */
  /* ANGLE ADJUSTMENT DOES NOT EFFECT THIS VALUE */
  public double get180Angle() {
    return gyro.getYaw();
  }
  
  /* Resets gyro angle*/
  public void resetGyroYaw() {
    gyro.reset();
  }

  /* Calibrates the gyro */
  /* ROBOT MUST BE STILL WHILE CALIBRATING */
  public void calibrateGyro() {
    gyro.calibrate();
    System.out.println("- - - DO NOT MOVE ROBOT - - -");
  }

  public boolean isCalibrating() {
    if (gyro.isCalibrating() == true) {
      return true;
    }
    else {
      return false;
    }
  }
  
  /* Sets the gyro offset angle */
  /* If the gyro thinks right is forward, then the adjustment should be 90 */
  /* If the gyro thinks left is forward, then the adjustment should be -90 */ 
  /* If the gyro thinks backward is forward, then the adjustment should be 180 */ 
  public void setGyroAngleOffset(double adjustment) {
    gyro.setAngleAdjustment(adjustment);
  }

  public double getVelocityY() {
    double velocityY = -gyro.getVelocityX();
    return velocityY;
  }

  public double getVelocityX() {
    double velocityX = gyro.getVelocityY();
    return velocityX;
  }

  public double getAveragedVelocityY() {
    double averagedVelocityY = (getVelocityX() + swerveDrive.chassisYVelocity()) / 2;
    return averagedVelocityY;
  }

  public double getAveragedVelocityX() {
    double averagedVelocityX = (getVelocityY() - swerveDrive.chassisXVelocity()) / 2;
    return averagedVelocityX;
  }

  /* - - - VELOCITIES RELATIVE TO THE HUB - - - */

  // double gyroStrafe = (remote.getRightY()) * Math.sin(Math.toRadians(-swerveDrive.gyro.getAngle())) - (remote.getRightX()) * Math.cos(Math.toRadians(-swerveDrive.gyro.getAngle()));
  // double gyroForward = (remote.getRightY()) * Math.cos(Math.toRadians(-swerveDrive.gyro.getAngle())) + (remote.getRightX()) * Math.sin(Math.toRadians(-swerveDrive.gyro.getAngle()));
  public double getHubRelativeVelocityX() {
    double gyroXVelocity = (getAveragedVelocityY()) * Math.sin(Math.toRadians(limelight.getXCrosshairOffset())) - (getAveragedVelocityX()) * Math.cos(Math.toRadians(limelight.getXCrosshairOffset()));

    System.out.println("Chassis X Velocity: " + gyroXVelocity);

    return gyroXVelocity;
  }

  public double getHubRelativeVelocityY() {
    double gyroYVelocity = (getAveragedVelocityY()) * Math.cos(Math.toRadians(limelight.getXCrosshairOffset())) + (getAveragedVelocityX()) * Math.sin(Math.toRadians(limelight.getXCrosshairOffset()));

    System.out.println("Chassis Y Velocity: " + gyroYVelocity);

    return gyroYVelocity;
  }

  public double getHubRelativeVelocity() {
    double gyroVelocity = Math.sqrt(Math.pow(getHubRelativeVelocityX(), 2) + Math.pow(getHubRelativeVelocityY(), 2));
    return gyroVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
