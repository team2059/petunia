// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  WPI_TalonSRX extenderTalonSRX = new WPI_TalonSRX(ClimberConstants.climberExtenderTalonSRX);
  WPI_TalonSRX angleTalonSRX = new WPI_TalonSRX(ClimberConstants.climberAngleTalonSRX);

  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getExtenderMotorLocation() {
    return extenderTalonSRX.getSelectedSensorPosition();
  }

  public double getExtenderMotorVelocity() {
    return extenderTalonSRX.getSelectedSensorVelocity();
  }

  public double getAngleMotorLocation() {
    return angleTalonSRX.getSelectedSensorPosition();
  }

  public double getAngleMotorVelocity() {
    return angleTalonSRX.getSelectedSensorVelocity();
  }

  public void setExtenderMotorSpeed(double speed) {
    extenderTalonSRX.set(speed);
  }

  public void setAngleMotorSpeed(double speed) {
    angleTalonSRX.set(speed);
  }
}
