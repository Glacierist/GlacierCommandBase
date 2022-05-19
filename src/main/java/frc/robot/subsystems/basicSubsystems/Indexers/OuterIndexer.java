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

public class OuterIndexer extends SubsystemBase {
  private CANSparkMax outerIndexerMotor;
  private RelativeEncoder outerIndexerEncoder;
  private PIDController outerIndexerPIDController;
  private SimpleMotorFeedforward outerIndexerFeedforward;

  /** Creates a new OuterIndexer. */
  public OuterIndexer() {
    outerIndexerMotor = new CANSparkMax(Constants.outerIndexerID, MotorType.kBrushless);
    outerIndexerEncoder = outerIndexerMotor.getEncoder();

    outerIndexerPIDController = new PIDController(0, 0, 0);
    outerIndexerFeedforward = new SimpleMotorFeedforward(0, 0, 0);
  }


  public void runRPM(double RPM) {
    double velocity = outerIndexerEncoder.getVelocity();
    double PIDCalculate = outerIndexerPIDController.calculate(velocity, RPM);
    double feedforwardCalculate = outerIndexerFeedforward.calculate(velocity, RPM, 0.2);
    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    outerIndexerMotor.setVoltage(MathUtil.clamp(PIDCalculate + feedforwardCalculate, -12, 12));
  }

  public void runVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    outerIndexerMotor.setVoltage(voltage);
  }
  
  public void run() {
    outerIndexerMotor.set(Constants.intakeSpeed);
  }
  
  public void reverse() {
    outerIndexerMotor.set(-Constants.intakeSpeed);
  }

  public boolean getRunning() {
    if (Math.abs(outerIndexerEncoder.getVelocity()) > 0) {
      return true;
    }
    return false;
  }

  public void stop() {
    outerIndexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
