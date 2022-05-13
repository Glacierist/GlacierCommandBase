// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoAimHood;
import frc.robot.subsystems.AutoAimYaw;
import frc.robot.subsystems.SwerveInput;
import frc.robot.subsystems.basicSubsystems.shooterSubsystems.Limelight;

public class DriveLogicCommand extends CommandBase {
  private XboxController swerveController;
  private SwerveInput swerveInput;
  private AutoAimYaw autoAimYaw;
  private Limelight limelight;
  private AutoAimHood autoAimHood;
  
  /** Creates a new JoystickDriveCommand. */
  public DriveLogicCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveController = RobotContainer.swerveController;
    swerveInput = RobotContainer.swerveInput;
    autoAimYaw = RobotContainer.autoAimYaw;
    autoAimHood = RobotContainer.autoAimHood;
    limelight = RobotContainer.limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerveController.getBackButton() == true && limelight.hasTarget() == true) {
      autoAimYaw.aimYaw();
      autoAimHood.aimHood();
    }
    else {
      swerveInput.SwerveJoystick();
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
