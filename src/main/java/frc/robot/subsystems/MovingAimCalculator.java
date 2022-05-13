// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.basicSubsystems.Gyroscope;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Limelight;

public class MovingAimCalculator extends SubsystemBase {
  public Gyroscope gyro;
  public Limelight limelight;
  
  /** Creates a new MovingAimCalculator. */
  public MovingAimCalculator() {
    gyro = RobotContainer.gyro;
    limelight = RobotContainer.limelight;

  }

  public double turretYawAngle() {
    double yawAngle = Math.atan2(gyro.getHubRelativeVelocityY(), gyro.getHubRelativeVelocityX());
    return yawAngle;
  }

  public double turretHoodAngle() {
    double movingHubDistance = Math.sqrt(Math.pow(limelight.getTargetDistance() + gyro.getHubRelativeVelocityY(), 2) + Math.pow(gyro.getHubRelativeVelocityX(), 2));
    double movingHoodAngle = movingHubDistance * Constants.distanceToHoodAngle;
    return movingHoodAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
