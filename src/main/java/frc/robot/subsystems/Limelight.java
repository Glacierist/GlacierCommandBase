// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    return table.getEntry("tx").getDouble(0);
  }

  public double getYCrosshairOffset() {
    return table.getEntry("ty").getDouble(0);
  }

  public boolean hasTarget() {
    if ((int) table.getEntry("ty").getDouble(0) == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getDistance() {
    return (Constants.hubHeight - Constants.limelightHeight) / Math.tan(Math.toRadians(Constants.limelightAngle + getYCrosshairOffset()));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
