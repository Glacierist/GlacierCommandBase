// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveInput extends SubsystemBase {
  private XboxController swerveController;
  private SwerveDrivetrain swerveDrive;

  /** Creates a new SwerveInput. */
  public SwerveInput() {
    swerveController = RobotContainer.swerveController;
    swerveDrive = RobotContainer.swerveDrive;
  }

  public void SwerveJoystick() {
    if (Math.abs(swerveController.getRawAxis(0)) > 0.05 || Math.abs(swerveController.getRawAxis(1)) > 0.05 || Math.abs(swerveController.getRawAxis(2)) > 0.05) {
      swerveDrive.periodicModuleUpdate(swerveController.getRawAxis(0) * Constants.maxVelocityMultiplier, swerveController.getRawAxis(1) * Constants.maxVelocityMultiplier, swerveController.getRawAxis(2) * Constants.radiansPerSecondMultiplier);
    // }

    // if (Math.abs(swerveController.getRightY()) > 0.05 || Math.abs(swerveController.getRightX()) > 0.05 || Math.abs(swerveController.getLeftX()) > 0.05) {
    //   swerveDrive.periodicModuleUpdate(swerveController.getRightY() * Constants.maxVelocityMultiplier, swerveController.getRightX() * Constants.maxVelocityMultiplier, swerveController.getLeftX() * Constants.radiansPerSecondMultiplier);
    }
  }

  public void SwerveYawInput(double yawInput) {
    if (Math.abs(swerveController.getRawAxis(0)) > 0.05 || Math.abs(swerveController.getRawAxis(1)) > 0.05) {
      swerveDrive.periodicModuleUpdate(swerveController.getRawAxis(0) * Constants.maxVelocityMultiplier, swerveController.getRawAxis(1) * Constants.maxVelocityMultiplier, yawInput * Constants.radiansPerSecondMultiplier);
    }
  }

  public void SwerveAllInput(double forward, double strafe, double yaw) {
      swerveDrive.periodicModuleUpdate(forward * Constants.maxVelocityMultiplier, strafe * Constants.maxVelocityMultiplier, yaw * Constants.radiansPerSecondMultiplier);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
