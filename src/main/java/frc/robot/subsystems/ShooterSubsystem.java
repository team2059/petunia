// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.VisionShootCmd;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterSubsystem extends SubsystemBase {
  /*
   * This block creates two motor instances, the motor controller was a TalonSRX
   * and it takes in a the Motor Port
   */
  public static WPI_TalonSRX ballShooter = new WPI_TalonSRX(ShooterConstants.shooterMotorTalonSRX);
  public static WPI_TalonSRX indexMotor = new WPI_TalonSRX(ShooterConstants.intakeIndexerTalonSRX);

  public static WPI_TalonSRX oppositeFlywheel = new WPI_TalonSRX(ShooterConstants.oppositeFlywheelTalonSRX);

  public static DigitalInput ballChamberSensor = new DigitalInput(0);

  public static void setIndexSpeed(double speed) {
    if (ballShooter.getSelectedSensorVelocity() != 0) {
      new WaitCommand(1.25);
      indexMotor.set(speed);
    } else {
      indexMotor.set(0);
    }
  }

  public static WPI_TalonSRX getPrimary() {
    return ballShooter;
  }

  /*
   * Creates a new Shooter.
   */
  public ShooterSubsystem() {
    indexMotor.configFactoryDefault();
    indexMotor.setNeutralMode(NeutralMode.Brake);
    
    indexMotor.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    ballShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.ShooterConstants.kPIDLoopIdx,
        Constants.ShooterConstants.kTimeoutMs);

    /**
     * Phase sensor accordingly.
     * Positive Sensor Reading should match Green (blinking) Leds on Talon
     */
    ballShooter.setSensorPhase(true);

    // Resets all values to default
    ballShooter.configFactoryDefault();
    ballShooter.setNeutralMode(NeutralMode.Brake);

    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    ballShooter.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.selectProfileSlot(ShooterConstants.kSlotIdx, ShooterConstants.kPIDLoopIdx);

    ballShooter.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kF,
        Constants.ShooterConstants.kTimeoutMs);
    ballShooter.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kP,
        Constants.ShooterConstants.kTimeoutMs);
    ballShooter.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kI,
        Constants.ShooterConstants.kTimeoutMs);
    ballShooter.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kD,
        Constants.ShooterConstants.kTimeoutMs);

    /* Config sensor used for Primary PID [Velocity] */
    oppositeFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.ShooterConstants.kPIDLoopIdx,
        Constants.ShooterConstants.kTimeoutMs);

    // Resets all values to default
    oppositeFlywheel.configFactoryDefault();
    oppositeFlywheel.setNeutralMode(NeutralMode.Coast);

    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    oppositeFlywheel.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    oppositeFlywheel.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    oppositeFlywheel.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    oppositeFlywheel.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    oppositeFlywheel.selectProfileSlot(ShooterConstants.kSlotIdx, ShooterConstants.kPIDLoopIdx);

    oppositeFlywheel.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kF,
        Constants.ShooterConstants.kTimeoutMs);
    oppositeFlywheel.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kP,
        Constants.ShooterConstants.kTimeoutMs);
    oppositeFlywheel.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kI,
        Constants.ShooterConstants.kTimeoutMs);
    oppositeFlywheel.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kD,
        Constants.ShooterConstants.kTimeoutMs);

    /**
     * Phase sensor accordingly.
     * Positive Sensor Reading should match Green (blinking) Leds on Talon
     */
    oppositeFlywheel.setSensorPhase(false);
    oppositeFlywheel.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Primary ticks", ballShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Secondary velocity", oppositeFlywheel.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Is Ball in chamber", ballChamberSensor.get());
    SmartDashboard.putBoolean("Is spinning", ballShooter.getSelectedSensorVelocity() > 0);
  }

  public void autoLoader() {
    if (ballChamberSensor.get()) {
      indexMotor.set(0);
      ;
    } else {
      indexMotor.set(-1);
    }

  }

  public void setShooterVelocity(double velocity) {
    ballShooter.set(velocity);
  }
}