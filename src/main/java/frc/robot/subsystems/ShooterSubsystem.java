// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterSubsystem extends SubsystemBase {
  /*
   * This block creates two motor instances, the motor controller was a TalonSRX
   * and it takes in a the Motor Port
   */
  public static WPI_TalonSRX ballShooter = new WPI_TalonSRX(ShooterConstants.shooterMotorTalonSRX);
  public static WPI_TalonSRX indexMotor = new WPI_TalonSRX(ShooterConstants.intakeIndexerTalonSRX);
  // TODO change port
  DigitalInput chamberPhotoElectricSensor = new DigitalInput(1);

  public static void setIndexSpeed(double speed) {
    indexMotor.set(speed);
  }

  /*
   * Creates a new Shooter.
   */
  public ShooterSubsystem() {
    // Resets all values to default
    ballShooter.configFactoryDefault();
    ballShooter.setNeutralMode(NeutralMode.Coast);
    ballShooter.setSelectedSensorPosition(0);

    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    ballShooter.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.selectProfileSlot(ShooterConstants.kSlotIdx, ShooterConstants.kPIDLoopIdx);

    // TODO Configure Gain PID stuff (then uncomment)
    // /*
    // * ballShooter.config_kP(ShooterConstants.kPIDLoopIdx,
    // * ShooterConstants.kGains.kP, ShooterConstants.kCtreTimeoutMs);
    // * ballShooter.config_kI(ShooterConstants.kPIDLoopIdx,
    // * ShooterConstants.kGains.kI, ShooterConstants.kCtreTimeoutMs);
    // * ballShooter.config_kD(ShooterConstants.kPIDLoopIdx,
    // * ShooterConstants.kGains.kD, ShooterConstants.kCtreTimeoutMs);
    // * ballShooter.config_kF(ShooterConstants.kPIDLoopIdx,
    // * ShooterConstants.kGains.kF, ShooterConstants.kCtreTimeoutMs);
    // */
    indexMotor.configFactoryDefault();

    indexMotor.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    indexMotor.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    indexMotor.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    indexMotor.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    indexMotor.selectProfileSlot(ShooterConstants.kSlotIdx, ShooterConstants.kPIDLoopIdx);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ball Shooter Velocity", ballShooter.getSelectedSensorVelocity());
  }

  public double getShooterVelocity() {
    return ballShooter.getSelectedSensorVelocity();
  }

  public void setShooterVelocity(double velocity) {
    ballShooter.set(velocity);
  }
}