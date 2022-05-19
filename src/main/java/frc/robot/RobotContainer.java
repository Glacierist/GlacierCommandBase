// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Made by Glacierist

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveLogicCommand;
import frc.robot.subsystems.TurretAutoAim;
import frc.robot.subsystems.AimCalculator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveInput;
import frc.robot.subsystems.basicSubsystems.Gyro;
import frc.robot.subsystems.basicSubsystems.SwerveModule;
import frc.robot.subsystems.basicSubsystems.Intake.Pneumatics;
import frc.robot.subsystems.basicSubsystems.Shooter.Hood;
import frc.robot.subsystems.basicSubsystems.Shooter.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final AutonomousCommand autoCommand = new AutonomousCommand();

  public static SwerveModule swerveModule;
  public static SwerveDrivetrain swerveDrive;
  public static XboxController swerveController;
  public static XboxController alternateController;
  public static Limelight limelight;
  public static SwerveInput swerveInput;
  public static Gyro gyro;
  public static TurretAutoAim turretAutoAim;
  public static Hood hood;
  public static AimCalculator aimCalculator;
  public static Pneumatics pneumatics;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerveDrive = new SwerveDrivetrain();
    swerveController = new XboxController(Constants.swerveControllerPort);
    alternateController = new XboxController(Constants.alternateControllerPort);
    limelight = new Limelight();
    swerveInput = new SwerveInput();
    gyro = new Gyro(90);
    turretAutoAim = new TurretAutoAim();
    hood = new Hood(8);
    aimCalculator = new AimCalculator();
    pneumatics = new Pneumatics();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(swerveController, XboxController.Button.kStart.value).whileActiveContinuous(new DriveLogicCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
