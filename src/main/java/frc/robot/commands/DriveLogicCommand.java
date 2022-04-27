// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoAimYaw;
import frc.robot.subsystems.SwerveInput;

public class DriveLogicCommand extends CommandBase {
  private XboxController swerveController;
  private SwerveInput swerveInput;
  private AutoAimYaw autoAimYaw;
  
  /** Creates a new JoystickDriveCommand. */
  public DriveLogicCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveController = RobotContainer.swerveController;
    swerveInput = RobotContainer.swerveInput;
    autoAimYaw = RobotContainer.autoAimYaw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.swerveController.getBackButton() == false) {
      swerveInput.SwerveJoystick();
    }
    else {
      RobotContainer.autoAimYaw.TrackTarget();
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
