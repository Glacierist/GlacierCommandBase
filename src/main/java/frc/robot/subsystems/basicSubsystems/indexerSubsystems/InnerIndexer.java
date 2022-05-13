// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.indexerSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InnerIndexer extends SubsystemBase {
  private CANSparkMax innerIndexerMotor;

  /** Creates a new InnerIndexer. */
  public InnerIndexer() {
    innerIndexerMotor = new CANSparkMax(Constants.innerIndexerID, MotorType.kBrushless);
    
  }

  public void runIndexerRPM(double RPM) {
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    innerIndexerMotor.setVoltage(RPM / 473);
  }

  public void runInnerIndexerVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    innerIndexerMotor.setVoltage(voltage);
  }
  
  public void runInnerIndexer() {
    innerIndexerMotor.set(Constants.intakeSpeed);
  }
  
  public void reverseInnerIndexer() {
    innerIndexerMotor.set(-Constants.intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
