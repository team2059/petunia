// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.can.*;

public class BallCollecter extends SubsystemBase {

  WPI_TalonSRX collectorMotor = new WPI_TalonSRX(Constants.collectorMotorTalonSRX);

  /** Creates a new BallCollecter. */
  public BallCollecter() {

    // TODO figure out why we use super("BallCollecter");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCollecterMotorSpeed(double speed) {
    collectorMotor.set(speed);
  }

  public WPI_TalonSRX getCollecterMotorTalonSRX() {
    return collectorMotor;
  }

public void setCollectorMotor(double speed) {
}
}
