// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.Climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {
  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  /** Creates a new Climbers. */
  public Climbers() {
    leftClimberMotor = new CANSparkMax(Constants.leftClimberID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.rightClimberID, MotorType.kBrushless);

  }

  public void runClimberRPM(double RPM) {
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    leftClimberMotor.setVoltage(RPM / 473);
    rightClimberMotor.setVoltage(RPM / 473);
  }

  public void runClimberVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    leftClimberMotor.setVoltage(voltage);
    rightClimberMotor.setVoltage(-voltage);
  }

  public void extendClimber() {
    leftClimberMotor.set(Constants.climberSpeed);
    rightClimberMotor.set(-Constants.climberSpeed);
  }

  public void retractClimber() {
    leftClimberMotor.set(-Constants.climberSpeed);
    rightClimberMotor.set(Constants.climberSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
