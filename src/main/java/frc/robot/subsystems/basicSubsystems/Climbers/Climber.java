// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.Climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;
  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  private PIDController leftClimberPIDController;
  private PIDController rightClimberPIDController;
  private SimpleMotorFeedforward leftClimberFeedforward;
  private SimpleMotorFeedforward rightClimberFeedforward;

  /** Creates a new Climbers. */
  public Climber() {
    leftClimberMotor = new CANSparkMax(Constants.leftClimberID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.rightClimberID, MotorType.kBrushless);
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftClimberPIDController = new PIDController(0, 0, 0);
    rightClimberPIDController = new PIDController(0, 0, 0);
    leftClimberFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    rightClimberFeedforward = new SimpleMotorFeedforward(0, 0, 0);
  }

  public void runRPM(double RPM) {
    double leftVelocity = leftClimberEncoder.getVelocity();
    double rightVelocity = rightClimberEncoder.getVelocity();

    double leftPIDCalculate = leftClimberPIDController.calculate(leftVelocity, RPM);
    double rightPIDCalculate = leftClimberPIDController.calculate(rightVelocity, RPM);

    double leftFeedforwardCalculate = leftClimberFeedforward.calculate(leftVelocity, RPM, 0.2);
    double rightFeedforwardCalculate = leftClimberFeedforward.calculate(rightVelocity, RPM, 0.2);

    if (RPM > 5676) {
      RPM = 5676;
    }
    else if (RPM < -5676) {
      RPM = -5676;
    }
    leftClimberMotor.setVoltage(leftPIDCalculate + leftFeedforwardCalculate);
    rightClimberMotor.setVoltage(rightPIDCalculate + rightFeedforwardCalculate);
  }

  public void runVoltage(double voltage) {
    if (voltage > 12) {
      voltage = 12;
    }
    else if (voltage < -12) {
      voltage = -12;
    }
    leftClimberMotor.setVoltage(voltage);
    rightClimberMotor.setVoltage(-voltage);
  }

  public void extend() {
    leftClimberMotor.set(Constants.climberSpeed);
    rightClimberMotor.set(-Constants.climberSpeed);
  }

  public void retract() {
    leftClimberMotor.set(-Constants.climberSpeed);
    rightClimberMotor.set(Constants.climberSpeed);
  }

  public void stop() {
    leftClimberMotor.stopMotor();
    rightClimberMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
