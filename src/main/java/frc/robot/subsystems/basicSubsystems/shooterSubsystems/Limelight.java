// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.shooterSubsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private NetworkTable table;
  private NetworkTableEntry pipeline;
 
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight-sam");
    pipeline = table.getEntry("pipeline");
  }

  public double getXCrosshairOffset() {
    double xOffset = table.getEntry("tx").getDouble(0);
    if (xOffset > 26) {
      System.out.println("Limelight near max angle");
    }
    else if (xOffset == 0) {
      System.out.println("Limelight past max angle");
    }
    return xOffset;
  }

  public double getYCrosshairOffset() {
    double yCrosshairOffset = table.getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("Limelight Y Crosshair Offset", yCrosshairOffset);
    return yCrosshairOffset;
  }

  public boolean hasTarget() {
    if ((int) table.getEntry("ty").getDouble(0) == 1) {
      SmartDashboard.putBoolean("Limelight Has Target", true);
      return true;
    }
    else {
      SmartDashboard.putBoolean("Limelight Has Target", false);
      return false;
    }
  }

  public double getTargetDistance() {
    double targetDistance = (Constants.hubHeight - Constants.limelightHeight) / Math.tan(Math.toRadians(Constants.limelightAngle + getYCrosshairOffset()));
    SmartDashboard.putNumber("Limelight Target Distance", targetDistance);
    return targetDistance;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
