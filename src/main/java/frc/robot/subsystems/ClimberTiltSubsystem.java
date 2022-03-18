// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberTiltSubsystem extends SubsystemBase {
        /** Creates a new ClimberAngleSubsystem. */

        static WPI_TalonSRX climberLeftTiltTalonSRX = new WPI_TalonSRX(
                        Constants.ClimberTiltConstants.climberLeftTiltSRX);
        static WPI_TalonSRX climberRightTiltTalonSRX = new WPI_TalonSRX(
                        Constants.ClimberTiltConstants.climberRightTiltSRX);

        static WPI_TalonSRX hookTalonSRX = new WPI_TalonSRX(Constants.ClimberTiltConstants.hookTalonSRX);

        public static WPI_TalonSRX getClimberLeftTiltSRX() {
                return climberLeftTiltTalonSRX;
        }

        public static WPI_TalonSRX getClimberRightTiltSRX() {
                return climberRightTiltTalonSRX;
        }

        public static void stopMotors() {
                climberLeftTiltTalonSRX.set(0);
                climberRightTiltTalonSRX.set(0);
        }

        public static void reset() {
                climberLeftTiltTalonSRX.setSelectedSensorPosition(0);
                climberRightTiltTalonSRX.setSelectedSensorPosition(0);
        }

        public ClimberTiltSubsystem() {
                climberLeftTiltTalonSRX.setSelectedSensorPosition(0, Constants.ClimberTiltConstants.kPIDLoopIdx,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberRightTiltTalonSRX.setSelectedSensorPosition(0, Constants.ClimberTiltConstants.kPIDLoopIdx,
                                Constants.ClimberTiltConstants.kTimeoutMs);

                climberRightTiltTalonSRX.setNeutralMode(NeutralMode.Coast);
                climberRightTiltTalonSRX.setNeutralMode(NeutralMode.Coast);

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

                // talon 11 settings
                climberLeftTiltTalonSRX.setSensorPhase(true);
                climberLeftTiltTalonSRX.setInverted(true);

                // talon 12 settings
                climberRightTiltTalonSRX.setSensorPhase(true);
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

                climberLeftTiltTalonSRX.selectProfileSlot(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.kPIDLoopIdx);
                climberLeftTiltTalonSRX.config_kF(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kF,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberLeftTiltTalonSRX.config_kP(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kP,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberLeftTiltTalonSRX.config_kI(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kI,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberLeftTiltTalonSRX.config_kD(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kD,
                                Constants.ClimberTiltConstants.kTimeoutMs);

                /* Set acceleration and vcruise velocity - see documentation */
                // TUNE CRUISE VELOCITY WHEN TESTING
                climberLeftTiltTalonSRX.configMotionCruiseVelocity(50, Constants.ClimberTiltConstants.kTimeoutMs);
                climberLeftTiltTalonSRX.configMotionAcceleration(50.4, Constants.ClimberTiltConstants.kTimeoutMs);

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

                climberRightTiltTalonSRX.selectProfileSlot(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.kPIDLoopIdx);
                climberRightTiltTalonSRX.config_kF(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kF,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberRightTiltTalonSRX.config_kP(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kP,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberRightTiltTalonSRX.config_kI(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kI,
                                Constants.ClimberTiltConstants.kTimeoutMs);
                climberRightTiltTalonSRX.config_kD(Constants.ClimberTiltConstants.kSlotIdx,
                                Constants.ClimberTiltConstants.gains.kD,
                                Constants.ClimberTiltConstants.kTimeoutMs);

                /* Set acceleration and vcruise velocity - see documentation */
                // TUNE CRUISE VELOCITY WHEN TESTING
                climberRightTiltTalonSRX.configMotionCruiseVelocity(50, Constants.ClimberTiltConstants.kTimeoutMs);
                climberRightTiltTalonSRX.configMotionAcceleration(50.4, Constants.ClimberTiltConstants.kTimeoutMs);

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
                // SmartDashboard.putString("right mode",
                // climberRightTiltTalonSRX.getControlMode().toString());
                // SmartDashboard.putString("left mode",
                // climberLeftTiltTalonSRX.getControlMode().toString());

        }
}