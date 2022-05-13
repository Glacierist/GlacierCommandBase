// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.shooterSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  public CANSparkMax hoodMotor;
  
  public RelativeEncoder hoodEncoder;
  
  public PIDController hoodPIDController;
  public ArmFeedforward hoodFeedforwardController;

  /** Creates a new Hood. */
  public Hood(double defaultAngle) {
    hoodMotor = new CANSparkMax(Constants.hoodMotorID, MotorType.kBrushless);

    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPositionConversionFactor(Constants.motorToHoodConversion);

    hoodPIDController = new PIDController(0.004, 0, 0.001);
    hoodFeedforwardController = new ArmFeedforward(0.1, 0.1, 0.1);
  }

  public void setHoodAngle(double angle) {
    hoodMotor.setVoltage(hoodFeedforwardController.calculate(angle, 0) + hoodPIDController.calculate(hoodEncoder.getPosition(), angle));
  }

  public void home() {
    hoodMotor.setVoltage(4);
    if ((hoodEncoder.getVelocity() < 5) && hoodMotor.getOutputCurrent() > 1) {
      hoodMotor.setVoltage(0);
      hoodEncoder.setPosition(0);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
