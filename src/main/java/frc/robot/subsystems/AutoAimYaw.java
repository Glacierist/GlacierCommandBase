// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.basicSubsystems.Gyroscope;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Limelight;

public class AutoAimYaw extends SubsystemBase {
  private Limelight limelight;
  private PIDController yawPIDController;
  private SwerveInput swerveInput;
  private Gyroscope gyro;
  private MovingAimCalculator aimCalculator;

  /** Creates a new AutoAimYaw. */
  public AutoAimYaw() {
    limelight = RobotContainer.limelight;
    swerveInput = RobotContainer.swerveInput;
    gyro = RobotContainer.gyro;
    aimCalculator = RobotContainer.aimCalculator;


    yawPIDController = new PIDController(1.5, 0, 0.8);
    yawPIDController.setTolerance(2, 1);
  }

  public void aimYaw() {
    swerveInput.SwerveYawInput(yawPIDController.calculate(limelight.getXCrosshairOffset(), aimCalculator.turretYawAngle()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
