// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter extends SubsystemBase {
  WPI_TalonSRX ballShooter = new WPI_TalonSRX(ShooterConstants.kBallShooterPort);
  WPI_TalonSRX indexMotor = new WPI_TalonSRX(ShooterConstants.kIndexMotorPort);
  // TODO change port
  DigitalInput chamberPhotoElectricSensor = new DigitalInput(1);

  /** Creates a new Shooter. */
  public Shooter() {
    ballShooter.configFactoryDefault();
    ballShooter.setSensorPhase(true);
    // set constant timeout
    ballShooter.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.selectProfileSlot(ShooterConstants.kMotorSlotIdx, ShooterConstants.kLoopIdx);

    indexMotor.configFactoryDefault();
    indexMotor.setSensorPhase(true);
    // set constant timeout
    indexMotor.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    indexMotor.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    indexMotor.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    indexMotor.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    //Check about this line
    indexMotor.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ball Shooter Velocity", ballShooter.getSelectedSensorVelocity());
  }

  public double getShooterVelocity() {
    return ballShooter.getSelectedSensorVelocity();
  }

  public void setShooterVelocity() {

  }
}