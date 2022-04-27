// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax turnMotor;
  private CANSparkMax driveMotor;
  private RelativeEncoder turnEncoder;
  private RelativeEncoder driveEncoder;
  private double turnEncoder180;
  private PIDController turnPIDController;
  private PIDController drivePIDController;
  private SimpleMotorFeedforward driveFeedforward;
  
  public SwerveModule(int turnMotorID, int driveMotorID, int encoderPort) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnEncoder = turnMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    turnPIDController = new PIDController(0.00, 0.00, 0.00);
    drivePIDController = new PIDController(0.00, 0.00, 0.00);
    driveFeedforward = new SimpleMotorFeedforward(0.55493, 2.3014, 0.51488);

    /* Converts number of rotations to the angle of the module in radians */
    turnEncoder.setPositionConversionFactor(Constants.turnEncoderPositionConversion);
    /* Converts RPM of the motor to m/s of a 106mm (0.106m) wheel */
    driveEncoder.setVelocityConversionFactor(Constants.driveEncoderVelocityConversion);

    /* Considers -180 and 180 the same value so the module doesn't do a 360 to get to the next setpoint */
    turnPIDController.enableContinuousInput(-180, 180);
  }

  /* Sets a swerve module to a given angle in degrees and speed in m/s */
  public void setModule(Rotation2d angle, double speed) {
    /* PID controller and motion profile to turn the swerve module to a givien angle (degrees) */
    turnMotor.set(MathUtil.clamp(turnPIDController.calculate(get180Angle().getDegrees(), angle.getDegrees()), -0.4, 0.4));

    /* Feed Forward controller + PID controller: Uses system dynamics and PID to get a more responsive drivetrain */
    driveMotor.setVoltage(driveFeedforward.calculate(driveEncoder.getVelocity(), speed, 0.2) + drivePIDController.calculate(driveEncoder.getVelocity(), speed));
  }

  public Rotation2d get180Angle() {
    if (turnEncoder.getPosition() > 360) {
      turnEncoder180 = (turnEncoder.getPosition() % 360) - 180;
    }
    else if (turnEncoder.getPosition() < 0) {
      turnEncoder180 = (turnEncoder.getPosition() % 360) + 180;
    }
    else {
      turnEncoder180 = turnEncoder.getPosition() - 180;
    }
    return Rotation2d.fromDegrees(turnEncoder180);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
