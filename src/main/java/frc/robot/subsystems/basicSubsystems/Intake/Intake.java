// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeEncoder;
  private PIDController intakePIDController;
  private SimpleMotorFeedforward intakeFeedforward;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    intakePIDController = new PIDController(0, 0, 0);
    intakeFeedforward = new SimpleMotorFeedforward(0, 0, 0);
  }

  public void runRPM(double RPM) {
    double velocity = intakeEncoder.getVelocity();
    double PIDCalculate = intakePIDController.calculate(velocity, RPM);
    double feedforwardCalculate = intakeFeedforward.calculate(velocity, RPM, 0.2);
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    intakeMotor.setVoltage(MathUtil.clamp(PIDCalculate + feedforwardCalculate, -12, 12));
  }

  public void setVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    intakeMotor.setVoltage(voltage);
  }
  
  public void run() {
    intakeMotor.set(Constants.intakeSpeed);
  }
  
  public void reverse() {
    intakeMotor.set(-Constants.intakeSpeed);
  }

  public boolean getRunning() {
    if (Math.abs(intakeEncoder.getVelocity()) > 0) {
      return true;
    }
    return false;
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
