// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.indexerSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OuterIndexer extends SubsystemBase {
  private CANSparkMax outerIndexerMotor;

  /** Creates a new OuterIndexer. */
  public OuterIndexer() {
    outerIndexerMotor = new CANSparkMax(Constants.outerIndexerID, MotorType.kBrushless);
  }


  public void runOuterIndexerRPM(double RPM) {
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    outerIndexerMotor.setVoltage(RPM / 473);
  }

  public void runOuterIndexerVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    outerIndexerMotor.setVoltage(voltage);
  }
  
  public void runOuterIndexer() {
    outerIndexerMotor.set(Constants.intakeSpeed);
  }
  
  public void reverseOuterIndexer() {
    outerIndexerMotor.set(-Constants.intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
