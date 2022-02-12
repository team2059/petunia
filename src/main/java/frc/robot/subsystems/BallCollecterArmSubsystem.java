// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.*;

public class BallCollecterArmSubsystem extends SubsystemBase {

  // figure out mechanism (limit switch, encoder, hall effect, etc)

  WPI_TalonSRX ballCollecterArmTalonSRX = new WPI_TalonSRX(Constants.CollecterConstants.ballCollecterArmTalonSRX);

  public WPI_TalonSRX getBallCollecterArmTalonSRX() {
    return ballCollecterArmTalonSRX;
  }

  public void setCollecterArmSpeed(double speed) {
    ballCollecterArmTalonSRX.set(speed);
  }

  /** Creates a new BallCollecterArm. */
  public BallCollecterArmSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
