// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax turnMotor;
  private CANSparkMax driveMotor;
  private RelativeEncoder turnEncoder;
  private RelativeEncoder driveEncoder;
  private ProfiledPIDController turnPIDController;
  private ProfiledPIDController drivePIDController;
  private SimpleMotorFeedforward driveFeedForward;
  // private double turnEncoder180 = 0;

  public SwerveModule(int turnMotorID, int driveMotorID, int encoderPort) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    turnEncoder = turnMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    /* Converts number of rotations to the angle of the module in radians */
    turnEncoder.setPositionConversionFactor(360 * (1/11.6571));
    /* Converts RPM of the motor to m/s of a 106mm (0.106m) wheel */
    driveEncoder.setVelocityConversionFactor((1/5.3333) * (0.106 * Math.PI) * (1/60));

    /* Module turning PID, max velocity, and max acceleration */
    turnPIDController = new ProfiledPIDController(0.00, 0.00, 0.00, new TrapezoidProfile.Constraints(10, 20));
    /* Drive PID, max velocity, and max acceleration */
    drivePIDController = new ProfiledPIDController(0.00, 0.00, 0.00, new TrapezoidProfile.Constraints(10, 20));
    driveFeedForward = new SimpleMotorFeedforward(0.18458, 0.13445, 0.023122);

    /* Considers -180 and 180 the same value so the module doesn't do a 360 to get to the next setpoint */
    turnPIDController.enableContinuousInput(-180, 180);
  }

  /* Sets a swerve module to a given angle in degrees and speed in m/s */
  public void setModule(Rotation2d angle, double speed) {
    /* PID controller and motion profile to turn the swerve module to a givien angle (degrees) */
    turnMotor.set(turnPIDController.calculate(turnEncoder.getPosition(), angle.getDegrees()));
    /* PID controller and motion profile to move the robot at a specific velocity (m/s) */
    // driveMotor.set(drivePIDController.calculate(driveEncoder.getVelocity(), speed));
    /* Feed Forward controller + PID controller: Uses system dynamics and PID to get a more responsive drivetrain */
    driveMotor.setVoltage(driveFeedForward.calculate(speed) + drivePIDController.calculate(driveEncoder.getVelocity(), speed));
  }

  public Rotation2d getCurrentAngle() {
    // if (turnEncoder.getPosition() > 360) {
    //   turnEncoder180 = (turnEncoder.getPosition() % 360) - 180;
    // }
    // else if (turnEncoder.getPosition() < 0) {
    //   turnEncoder180 = (turnEncoder.getPosition() % 360) + 180;
    // }
    // else {
    //   turnEncoder180 = turnEncoder.getPosition() - 180;
    // }
    return Rotation2d.fromDegrees(turnEncoder.getPosition());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
