// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.basicSubsystems.Gyroscope;

public class VelocityCalculator extends SubsystemBase {
  private Gyroscope gyro;

  /** Creates a new VelocityCalculator. */
  public VelocityCalculator() {
    gyro = RobotContainer.gyro;
  }

  public void VelocityCalculation() {
    // Use the gyro's X and Y integrated position to offset the limelight distance values and 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
