// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
  private AHRS gyro;

  /** Creates a new Gyro. */
  public Gyroscope(double offset) {
    
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
  
  /* Sets the gyro offset angle */
  /* If the gyro thinks right is forward, then the adjustment should be 90 */
  /* If the gyro thinks left is forward, then the adjustment should be -90 */ 
  /* If the gyro thinks backward is forward, then the adjustment should be 180 */ 
  public void setGyroAngleOffset(double adjustment) {
    gyro.setAngleAdjustment(adjustment);
  }

  public double getChassisVelocityX() {
    double chassisVelocityX = gyro.getVelocityY(); /* Because the gyro is 90 degrees from straight */
    return chassisVelocityX;
  }

  public double getChassisVelocityY() {
    double chassisVelocityY = -gyro.getVelocityX(); /* Because the gyro is 90 degrees from straight */
    return chassisVelocityY;
  }

  public double getChassisVelocity() {
    double chassisVelocity = Math.sqrt(Math.pow(getChassisVelocityX(), 2) + Math.pow(getChassisVelocityY(), 2));
    return chassisVelocity;
  }

  public double getChassisTheta() {
    double chassisTheta = Math.atan2(getChassisVelocityY(), getChassisVelocityX());
    return chassisTheta;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}