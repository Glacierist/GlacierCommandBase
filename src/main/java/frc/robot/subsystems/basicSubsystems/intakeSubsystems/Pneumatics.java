// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.basicSubsystems.intakeSubsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private PneumaticHub pneumaticsHub;
  private DoubleSolenoid solenoid;

  /** Creates a new Pneumatics. */
  public Pneumatics() {
    pneumaticsHub = new PneumaticHub(50);
    pneumaticsHub.enableCompressorDigital();
    solenoid = pneumaticsHub.makeDoubleSolenoid(14, 15);
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setSolenoid(boolean position) {
    if (position == true) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    else if (position == false) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
