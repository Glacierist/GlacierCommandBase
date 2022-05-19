// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private CANSparkMax leftFlywheelMotor;
  private CANSparkMax rightFlywheelMotor;

  private RelativeEncoder leftFlywheelEncoder;
  private RelativeEncoder rightFlywheelEncoder;

  // private ProfiledPIDController flywheelPIDController;
  private SimpleMotorFeedforward leftFlywheelFeedforward;
  private SimpleMotorFeedforward rightFlywheelFeedforward;

  /** Creates a new Flywheel. */
  public Flywheel() {
    leftFlywheelMotor = new CANSparkMax(Constants.leftFlywheelID, MotorType.kBrushless);
    rightFlywheelMotor = new CANSparkMax(Constants.rightFlywheelID, MotorType.kBrushless);
    rightFlywheelMotor.setInverted(true);

    leftFlywheelEncoder = leftFlywheelMotor.getEncoder();
    rightFlywheelEncoder = rightFlywheelMotor.getEncoder();

    leftFlywheelMotor.enableVoltageCompensation(12);
    rightFlywheelMotor.enableVoltageCompensation(12);

    // flywheelPIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(10, 20));
    leftFlywheelFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    rightFlywheelFeedforward = new SimpleMotorFeedforward(0, 0, 0);
  }

  public void setRPM(double RPM) {
    double leftVelocity = leftFlywheelEncoder.getVelocity();
    double rightVelocity = rightFlywheelEncoder.getVelocity();
    double leftFeedforwardCalculate = leftFlywheelFeedforward.calculate(leftVelocity, RPM, 1);
    double rightFeedforwardCalculate = rightFlywheelFeedforward.calculate(rightVelocity, RPM, 1); 
    if (RPM > 5676) {
      RPM = 5676;
    }
    leftFlywheelMotor.setVoltage(MathUtil.clamp(leftFeedforwardCalculate, -12, 12));
    rightFlywheelMotor.setVoltage(MathUtil.clamp(rightFeedforwardCalculate, -12, 12));
  }

  public void setTipSpeed(double velocity) {
    double leftVelocity = leftFlywheelEncoder.getVelocity() * Constants.RPMtoFlywheelTipSpeed;
    double rightVelocity = rightFlywheelEncoder.getVelocity() * Constants.RPMtoFlywheelTipSpeed;
    double leftFeedforwardCalculate = leftFlywheelFeedforward.calculate(leftVelocity, velocity, 1);
    double rightFeedforwardCalculate = rightFlywheelFeedforward.calculate(rightVelocity, velocity, 1);
    if (velocity > 45.2935) {
      velocity = 45.2935;
    }
    leftFlywheelMotor.setVoltage(MathUtil.clamp(leftFeedforwardCalculate, -12, 12));
    rightFlywheelMotor.setVoltage(MathUtil.clamp(rightFeedforwardCalculate, -12, 12));
  }

  public double getAvgRPM() {
    double avgRPM = (leftFlywheelEncoder.getVelocity() + rightFlywheelEncoder.getVelocity()) / 2;
    return avgRPM;
  }

  public double getAvgTipVelocity() {
    double avgTipVelocity = (leftFlywheelEncoder.getVelocity() * Constants.RPMtoFlywheelTipSpeed + rightFlywheelEncoder.getVelocity() * Constants.RPMtoFlywheelTipSpeed) / 2;
    return avgTipVelocity;
  }

  public double getLeftRPM() {
    return leftFlywheelEncoder.getVelocity();
  }

  public double getRightRPM() {
    return -rightFlywheelEncoder.getVelocity();
  }

  public double getTipSpeed() {
    return getAvgRPM() * Constants.RPMtoFlywheelTipSpeed;
  }

  public boolean getLeftRunning() {
    if (Math.abs(leftFlywheelMotor.get()) > 0) {
      return true;
    }
    return false;
  }

  public boolean getRightRunning() {
    if (Math.abs(rightFlywheelMotor.get()) > 0) {
      return true;
    }
    return false;
  }

  public void stop() {
    leftFlywheelMotor.stopMotor();
    rightFlywheelMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
