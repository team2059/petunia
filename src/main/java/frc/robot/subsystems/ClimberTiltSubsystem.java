// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberTiltSubsystem extends SubsystemBase {
    /** Creates a new ClimberAngleSubsystem. */

    static WPI_TalonSRX climberLeftTiltTalonSRX = new WPI_TalonSRX(Constants.ClimberTiltConstants.climberLeftTiltSRX);
    static WPI_TalonSRX climberRightTiltTalonSRX = new WPI_TalonSRX(Constants.ClimberTiltConstants.climberRightTiltSRX);

    public static WPI_TalonSRX getClimberLeftTiltSRX() {
        return climberLeftTiltTalonSRX;
    }

    public static WPI_TalonSRX getClimberRightTiltSRX() {
        return climberRightTiltTalonSRX;
    }

    public ClimberTiltSubsystem() {
        climberLeftTiltTalonSRX.setSelectedSensorPosition(0, Constants.ClimberTiltConstants.kPIDLoopIdx,
                Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.setSelectedSensorPosition(0, Constants.ClimberTiltConstants.kPIDLoopIdx,
                Constants.ClimberTiltConstants.kTimeoutMs);

        /* Factory default hardware to prevent unexpected behavior */
        climberLeftTiltTalonSRX.configFactoryDefault();

        /* Configure Sensor Source for Pirmary PID */
        climberLeftTiltTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                Constants.ClimberTiltConstants.kPIDLoopIdx,
                Constants.ClimberTiltConstants.kTimeoutMs);

        /*
         * set deadband to super small 0.001 (0.1 %).
         * The default deadband is 0.04 (4 %)
         */
        climberLeftTiltTalonSRX.configNeutralDeadband(0.001, Constants.ClimberTiltConstants.kTimeoutMs);

        /**
         * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
         * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
         * sensor to have positive increment when driving Talon Forward (Green LED)
         */
        climberLeftTiltTalonSRX.setSensorPhase(false);
        climberLeftTiltTalonSRX.setInverted(false);

        /**
         * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
         * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
         * sensor to have positive increment when driving Talon Forward (Green LED)
         */
        climberRightTiltTalonSRX.setSensorPhase(false);
        climberRightTiltTalonSRX.setInverted(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        climberLeftTiltTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
                Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
                Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set the peak and nominal outputs */
        climberLeftTiltTalonSRX.configNominalOutputForward(0, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.configNominalOutputReverse(0, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.configPeakOutputForward(1, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.configPeakOutputReverse(-1, Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        /* TODO TUNE PID VALUES */
        climberLeftTiltTalonSRX.selectProfileSlot(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.kPIDLoopIdx);
        climberLeftTiltTalonSRX.config_kF(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.masterGains.kF, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.config_kP(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.masterGains.kP, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.config_kI(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.masterGains.kI, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.config_kD(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.masterGains.kD, Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
        // TUNE CRUISE VELOCITY WHEN TESTING
        climberLeftTiltTalonSRX.configMotionCruiseVelocity(148, Constants.ClimberTiltConstants.kTimeoutMs);
        climberLeftTiltTalonSRX.configMotionAcceleration(147.6, Constants.ClimberTiltConstants.kTimeoutMs);

        climberLeftTiltTalonSRX.configFeedbackNotContinuous(true, Constants.ClimberTiltConstants.kTimeoutMs);

        // configure current limits
        climberLeftTiltTalonSRX.configPeakCurrentLimit(30);
        climberLeftTiltTalonSRX.configPeakCurrentDuration(150);

        // takes in amps
        climberLeftTiltTalonSRX.configContinuousCurrentLimit(20);

        // set integral zone
        climberLeftTiltTalonSRX.config_IntegralZone(Constants.ClimberTiltConstants.kSlotIdx, 3);

        /* Factory default hardware to prevent unexpected behavior */
        climberRightTiltTalonSRX.configFactoryDefault();

        /* Configure Sensor Source for Pirmary PID */
        climberRightTiltTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                Constants.ClimberTiltConstants.kPIDLoopIdx,
                Constants.ClimberTiltConstants.kTimeoutMs);

        /*
         * set deadband to super small 0.001 (0.1 %).
         * The default deadband is 0.04 (4 %)
         */
        climberRightTiltTalonSRX.configNeutralDeadband(0.001, Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        climberRightTiltTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
                Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
                Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set the peak and nominal outputs */
        climberRightTiltTalonSRX.configNominalOutputForward(0, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.configNominalOutputReverse(0, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.configPeakOutputForward(1, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.configPeakOutputReverse(-1, Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        /* TODO TUNE PID VALUES */
        climberRightTiltTalonSRX.selectProfileSlot(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.kPIDLoopIdx);
        climberRightTiltTalonSRX.config_kF(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.followerGains.kF, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.config_kP(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.followerGains.kP, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.config_kI(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.followerGains.kI, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.config_kD(Constants.ClimberTiltConstants.kSlotIdx,
                Constants.ClimberTiltConstants.followerGains.kD, Constants.ClimberTiltConstants.kTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
        // TUNE CRUISE VELOCITY WHEN TESTING
        climberRightTiltTalonSRX.configMotionCruiseVelocity(148, Constants.ClimberTiltConstants.kTimeoutMs);
        climberRightTiltTalonSRX.configMotionAcceleration(147.6, Constants.ClimberTiltConstants.kTimeoutMs);

        climberRightTiltTalonSRX.configFeedbackNotContinuous(true, Constants.ClimberTiltConstants.kTimeoutMs);

        // configure current limits
        climberRightTiltTalonSRX.configPeakCurrentLimit(30);
        climberRightTiltTalonSRX.configPeakCurrentDuration(150);

        // takes in amps
        climberRightTiltTalonSRX.configContinuousCurrentLimit(20);

        // set integral zone
        climberRightTiltTalonSRX.config_IntegralZone(Constants.ClimberTiltConstants.kSlotIdx, 3);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left tilt", climberLeftTiltTalonSRX.getSelectedSensorPosition());
        SmartDashboard.putNumber("right tilt", climberRightTiltTalonSRX.getSelectedSensorPosition());

    }
}