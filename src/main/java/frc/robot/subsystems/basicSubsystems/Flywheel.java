// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private CANSparkMax leftFlywheelMotor;
  private CANSparkMax rightFlywheelMotor;

  private RelativeEncoder leftFlywheelEncoder;
  private RelativeEncoder rightFlywheelEncoder;

  private ProfiledPIDController flywheelPIDController;
  private SimpleMotorFeedforward flywheelFeedforwardController;

  /** Creates a new Flywheel. */
  public Flywheel() {
    leftFlywheelMotor = new CANSparkMax(Constants.leftFlywheelID, MotorType.kBrushless);
    rightFlywheelMotor = new CANSparkMax(Constants.rightFlywheelID, MotorType.kBrushless);
    rightFlywheelMotor.setInverted(true);

    leftFlywheelEncoder = leftFlywheelMotor.getEncoder();
    rightFlywheelEncoder = rightFlywheelMotor.getEncoder();

    leftFlywheelMotor.enableVoltageCompensation(12);
    rightFlywheelMotor.enableVoltageCompensation(12);

    flywheelPIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(10, 20));
    flywheelFeedforwardController = new SimpleMotorFeedforward(0, 0, 0);
  }

  public void setFlywheelRPM(double RPM) {
    if (RPM > 5676) {
      RPM = 5676;
    }
    leftFlywheelMotor.setVoltage(flywheelFeedforwardController.calculate(leftFlywheelEncoder.getVelocity(), RPM / 473, 0.6));
    rightFlywheelMotor.setVoltage(flywheelFeedforwardController.calculate(rightFlywheelEncoder.getVelocity(), RPM / -473, 0.6));
  }

  public void setFlywheelTipSpeed(double velocity) {
    if (velocity > 45.2935) {
      velocity = 45.2935;
    }
    leftFlywheelMotor.setVoltage(flywheelFeedforwardController.calculate(leftFlywheelEncoder.getVelocity() * Constants.RPMtoFlywheelTipSpeed, velocity, 0.6));
    rightFlywheelMotor.setVoltage(flywheelFeedforwardController.calculate(rightFlywheelEncoder.getVelocity() * Constants.RPMtoFlywheelTipSpeed, velocity, 0.6));
  }

  public double getAvgFlywheelRPM() {
    double avgRPM = (leftFlywheelEncoder.getVelocity() + rightFlywheelEncoder.getVelocity()) / 2;
    return avgRPM;
  }

  public double getLeftFlywheelRPM() {
    return leftFlywheelEncoder.getVelocity();
  }

  public double getRightFlywheelRPM() {
    return -rightFlywheelEncoder.getVelocity();
  }

  public double getFlywheelTipSpeed() {
    return getAvgFlywheelRPM() * Constants.RPMtoFlywheelTipSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
