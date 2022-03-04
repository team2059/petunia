// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PIDShootCmd;
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

  public static DigitalInput ballChamberSensor = new DigitalInput(4);
  // TODO change port
  // DigitalInput chamberPhotoElectricSensor = new DigitalInput(1);

  public DigitalInput getBallChamberSensor() {
    return ballChamberSensor;
  }

  public static void setIndexSpeed(double speed) {
    indexMotor.set(speed);
  }

  public static BooleanSupplier getBallStatus() {
    return () -> ballChamberSensor.get();
  }

  /*
   * Creates a new Shooter.
   */
  public ShooterSubsystem() {
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
    ballShooter.setNeutralMode(NeutralMode.Coast);
    ballShooter.setSelectedSensorPosition(0);

    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    ballShooter.configNominalOutputForward(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configNominalOutputReverse(0, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputForward(1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.configPeakOutputReverse(-1, ShooterConstants.kCtreTimeoutMs);
    ballShooter.selectProfileSlot(ShooterConstants.kSlotIdx, ShooterConstants.kPIDLoopIdx);

    indexMotor.configFactoryDefault();
    ballShooter.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kF,
        Constants.ShooterConstants.kTimeoutMs);
    ballShooter.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kP,
        Constants.ShooterConstants.kTimeoutMs);
    ballShooter.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kI,
        Constants.ShooterConstants.kTimeoutMs);
    ballShooter.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kD,
        Constants.ShooterConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ball Shooter Velocity", ballShooter.getSelectedSensorVelocity());

    // if (ballChamberSensor.get()) {
    // setIndexSpeed(0);
    // }
    // setIndexSpeed(-0.66);
  }

  public void autoLoader(double speed) {
    if (ballChamberSensor.get()) {
      setShooterVelocity(0);
    } else {
      setShooterVelocity(speed);
    }
  }

  public double getShooterVelocity() {
    return ballShooter.getSelectedSensorVelocity();
  }

  public void setShooterVelocity(double velocity) {
    ballShooter.set(velocity);
  }
}