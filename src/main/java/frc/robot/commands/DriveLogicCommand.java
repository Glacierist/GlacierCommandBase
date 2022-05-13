// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretAutoAim;
import frc.robot.subsystems.SwerveInput;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Limelight;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Hood;

public class DriveLogicCommand extends CommandBase {
  private XboxController swerveController;
  private SwerveInput swerveInput;
  private Limelight limelight;
  private TurretAutoAim turretAutoAim;
  private Hood hood;
  
  /** Creates a new JoystickDriveCommand. */
  public DriveLogicCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveController = RobotContainer.swerveController;
    swerveInput = RobotContainer.swerveInput;
    turretAutoAim = RobotContainer.turretAutoAim;
    limelight = RobotContainer.limelight;
    hood = RobotContainer.hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.home();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerveController.getBackButton() == true && limelight.hasTarget() == true) {
      turretAutoAim.aimYaw();
      turretAutoAim.aimHood();
      SmartDashboard.putBoolean("Turret Running", true);
    }
    else {
      swerveInput.SwerveJoystick();
      SmartDashboard.putBoolean("Turret Running", false);
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
