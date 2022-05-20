// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.basicSubsystems.Gyro;

public class DrivetrainInverseKinematics extends SubsystemBase {
  private SwerveDrivetrain swerveDrive;
  private SwerveInput swerveInput;
  private Gyro gyro;

  /** Creates a new DrivetrainInverseKinematics. */
  public DrivetrainInverseKinematics() {
    swerveDrive = RobotContainer.swerveDrive;
    swerveInput = RobotContainer.swerveInput;
    gyro = RobotContainer.gyro;

  }

  public boolean setNextPosition(double positionX, double positionY, double rotation) {
    boolean withinRangeX = withinRange(swerveDrive.getXPose(), positionX, 0.1);
    boolean withinRangeY = withinRange(swerveDrive.getYPose(), positionY, 0.1);
    boolean withinRangeYaw = withinRange(-gyro.getTotalAngleDegrees(), rotation, 2);
    while (withinRangeX && withinRangeY && withinRangeYaw) {
      swerveInput.SwerveAllInput(swerveDrive.getYPose(), positionY, swerveDrive.getXPose(), positionX, gyro.getTotalAngleDegrees() % 360, rotation);
    }
    return true;
  }

  public boolean withinRange(double measurement, double check, double range) {
    if (measurement > (check - range) && measurement < (check + range)) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
