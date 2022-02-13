// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

public class BallCollecterArmSubsystem extends SubsystemBase {

  // figure out mechanism (limit switch, encoder, hall effect, etc)

  static WPI_TalonSRX ballCollecterArmTalonSRX = new WPI_TalonSRX(
      Constants.CollecterConstants.ballCollecterArmTalonSRX);

  public static WPI_TalonSRX getBallCollecterArmTalonSRX() {
    return ballCollecterArmTalonSRX;
  }

  public void setCollecterArmSpeed(double speed) {
    ballCollecterArmTalonSRX.set(speed);
  }

  /** Creates a new BallCollecterArm. */
  public BallCollecterArmSubsystem() {

    /* Factory default hardware to prevent unexpected behavior */
    ballCollecterArmTalonSRX.configFactoryDefault();
    

    /* Configure Sensor Source for Pirmary PID */
    ballCollecterArmTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
        Constants.CollecterArmConstants.kPIDLoopIdx,
        Constants.CollecterArmConstants.kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    ballCollecterArmTalonSRX.configNeutralDeadband(0.001, Constants.CollecterArmConstants.kTimeoutMs);

    /**
     * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    ballCollecterArmTalonSRX.setSensorPhase(false);
    ballCollecterArmTalonSRX.setInverted(false);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    ballCollecterArmTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
        Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
        Constants.CollecterArmConstants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    ballCollecterArmTalonSRX.configNominalOutputForward(0, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configNominalOutputReverse(0, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configPeakOutputForward(1, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configPeakOutputReverse(-1, Constants.CollecterArmConstants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    ballCollecterArmTalonSRX.selectProfileSlot(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kPIDLoopIdx);
    ballCollecterArmTalonSRX.config_kF(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kF, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.config_kP(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kP, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.config_kI(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kI, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.config_kD(Constants.CollecterArmConstants.kSlotIdx,
        Constants.CollecterArmConstants.kGains.kD, Constants.CollecterArmConstants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    ballCollecterArmTalonSRX.configMotionCruiseVelocity(973, Constants.CollecterArmConstants.kTimeoutMs);
    ballCollecterArmTalonSRX.configMotionAcceleration(972.75, Constants.CollecterArmConstants.kTimeoutMs);

    ballCollecterArmTalonSRX.configFeedbackNotContinuous(true, Constants.CollecterArmConstants.kTimeoutMs);

    // Configure current limits
    ballCollecterArmTalonSRX.configPeakCurrentLimit(30);
    ballCollecterArmTalonSRX.configPeakCurrentDuration(150);

    // takes in AMPS
    ballCollecterArmTalonSRX.configContinuousCurrentLimit(20);

    // integral zone
    ballCollecterArmTalonSRX.config_IntegralZone(Constants.CollecterArmConstants.kSlotIdx, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("talon encoder value", ballCollecterArmTalonSRX.getSelectedSensorPosition());

    

  }
}
