// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.basicSubsystems.Gyroscope;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Hood;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Limelight;

public class AutoAimHood extends SubsystemBase {
  private Limelight limelight;
  private Hood hood;
  private Gyroscope gyro;
  private MovingAimCalculator aimCalculator;

  /** Creates a new AutoAimHood. */
  public AutoAimHood() {
    limelight = RobotContainer.limelight;
    hood = RobotContainer.hood;
    gyro = RobotContainer.gyro;
    aimCalculator = RobotContainer.aimCalculator;

  }

  public void aimHood() {
    hood.setHoodAngle(aimCalculator.turretHoodAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
