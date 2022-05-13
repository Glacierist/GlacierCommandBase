// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.intakeSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);

  }

  public void runIntakeRPM(double RPM) {
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    intakeMotor.setVoltage(RPM / 473);
  }

  public void runIntakeVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    intakeMotor.setVoltage(voltage);
  }
  
  public void runIntake() {
    intakeMotor.set(Constants.intakeSpeed);
  }
  
  public void reverseIntake() {
    intakeMotor.set(-Constants.intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
