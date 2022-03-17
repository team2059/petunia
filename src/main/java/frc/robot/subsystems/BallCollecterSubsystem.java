// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.*;

public class BallCollecterSubsystem extends SubsystemBase {

  WPI_TalonSRX collectorMotor = new WPI_TalonSRX(Constants.CollecterConstants.collectorMotorTalonSRX);

  // TODO fill out port num
  WPI_TalonSRX collecterIndexer = new WPI_TalonSRX(13);

  /** Creates a new BallCollecter. */
  public BallCollecterSubsystem() {
    // TODO figure out why we use super("BallCollecter");
    // TODO set motor inversion
    collectorMotor.setInverted(false);

    // TODO set inversions
    collecterIndexer.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setSpeed(double speed) {
    collectorMotor.set(speed);
  }

  public WPI_TalonSRX getCollecterMotorTalonSRX() {
    return collectorMotor;
  }

  public WPI_TalonSRX getCollecterIndexer() {
    return collecterIndexer;
  }

}
