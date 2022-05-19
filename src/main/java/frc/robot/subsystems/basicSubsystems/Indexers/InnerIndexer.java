// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.Indexers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InnerIndexer extends SubsystemBase {
  private CANSparkMax innerIndexerMotor;
  private RelativeEncoder innerIndexerEncoder;
  private PIDController innerIndexerPIDController;
  private SimpleMotorFeedforward innerIndexeFeedforward;

  /** Creates a new InnerIndexer. */
  public InnerIndexer() {
    innerIndexerMotor = new CANSparkMax(Constants.innerIndexerID, MotorType.kBrushless);

    innerIndexerEncoder = innerIndexerMotor.getEncoder();

    innerIndexerPIDController = new PIDController(0, 0, 0);
    innerIndexeFeedforward = new SimpleMotorFeedforward(0, 0, 0);
  }

  public void runRPM(double RPM) {
    double velocity = innerIndexerEncoder.getVelocity();
    double PIDCalculate = innerIndexerPIDController.calculate(velocity, RPM);
    double feedforwardCalculate = innerIndexeFeedforward.calculate(velocity, RPM, 0.2);
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    innerIndexerMotor.setVoltage(MathUtil.clamp(PIDCalculate + feedforwardCalculate, -12, 12));
  }

  public void runVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    innerIndexerMotor.setVoltage(voltage);
  }
  
  public void run() {
    innerIndexerMotor.set(Constants.intakeSpeed);
  }
  
  public void reverse() {
    innerIndexerMotor.set(-Constants.intakeSpeed);
  }

  public boolean getRunning() {
    if (Math.abs(innerIndexerEncoder.getVelocity()) > 0) {
      return true;
    } 
    return false;
  }

  public void stop() {
    innerIndexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
