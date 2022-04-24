// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  private XboxController swerveController;
  private SwerveDrivetrain swerveDrive;

  public DriveCommand(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    swerveController = RobotContainer.swerveController;
    swerveDrive = RobotContainer.swerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerveController.getRawAxis(0)) > 0.05 || Math.abs(swerveController.getRawAxis(1)) > 0.05 || Math.abs(swerveController.getRawAxis(2)) > 0.05) {
      swerveDrive.periodicModuleUpdate(swerveController.getRawAxis(0) * Constants.maxVelocityMultiplier, swerveController.getRawAxis(1) * Constants.maxVelocityMultiplier, swerveController.getRawAxis(2) * Constants.radiansPerSecondMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
