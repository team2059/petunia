// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberExtenderSubsystem extends SubsystemBase {

        /** Creates a new ClimberSubsystem. */

        public static DigitalInput firstHangerLight = new DigitalInput(1);
        public static DigitalInput secondHangerLight = new DigitalInput(3);

        static WPI_TalonSRX climberLeftExtendSRX = new WPI_TalonSRX(
                        Constants.ClimberExtendConstants.climberLeftExtendSRX);
        static WPI_TalonSRX climberRightExtendSRX = new WPI_TalonSRX(
                        Constants.ClimberExtendConstants.climberRightExtendSRX);

        // static DigitalInput leftSensor = new DigitalInput(0);
        // static DigitalInput rightSensor = new DigitalInput(1);

        // boolean leftStatus = leftSensor.get();
        // boolean rightStatus = rightSensor.get();

        public static WPI_TalonSRX getClimberRightExtendSRX() {
                return climberRightExtendSRX;
        }

        public static WPI_TalonSRX getClimberLeftExtendSRX() {
                return climberLeftExtendSRX;
        }

        public static void stopMotors() {
                climberLeftExtendSRX.set(0);
                climberRightExtendSRX.set(0);
        }

        public ClimberExtenderSubsystem() {
                // if (leftStatus == false && rightStatus == false) {
                /* Zero the sensor once on robot boot up */
                climberLeftExtendSRX.setSelectedSensorPosition(0, Constants.ClimberExtendConstants.kPIDLoopIdx,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                /* Zero the sensor once on robot boot up */
                climberRightExtendSRX.setSelectedSensorPosition(0, Constants.ClimberExtendConstants.kPIDLoopIdx,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                // }

                /* Factory default hardware to prevent unexpected behavior */
                climberRightExtendSRX.configFactoryDefault();
                climberLeftExtendSRX.configFactoryDefault();

                // climberLeftExtendSRX.follow(climberRightExtendSRX, FollowerType.AuxOutput1);
                // _talon.setNeutralMode(NeutralMode.Brake);
                // _talon.setSelectedSensorPosition(0);

                /* Configure Sensor Source for Pirmary PID */
                climberRightExtendSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                Constants.ClimberExtendConstants.kPIDLoopIdx,
                                Constants.ClimberExtendConstants.kTimeoutMs);

                climberRightExtendSRX.configNeutralDeadband(0.001, Constants.ClimberExtendConstants.kTimeoutMs);

                // talon 9 settings
                climberLeftExtendSRX.setSensorPhase(true);
                climberLeftExtendSRX.setInverted(false);

                // talon 10 settings
                climberRightExtendSRX.setSensorPhase(true);
                climberRightExtendSRX.setInverted(true);

                /* Set relevant frame periods to be at least as fast as periodic rate */
                climberRightExtendSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
                                Constants.ClimberExtendConstants.kTimeoutMs);

                /* Set the peak and nominal outputs */
                climberRightExtendSRX.configNominalOutputForward(0, Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.configNominalOutputReverse(0, Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.configPeakOutputForward(1, Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.configPeakOutputReverse(-1, Constants.ClimberExtendConstants.kTimeoutMs);

                /* Set Motion Magic gains in slot0 - see documentation */
                climberRightExtendSRX.selectProfileSlot(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.kPIDLoopIdx);
                climberRightExtendSRX.config_kF(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kF,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.config_kP(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kP,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.config_kI(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kI,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.config_kD(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kD,
                                Constants.ClimberExtendConstants.kTimeoutMs);

                /* Set acceleration and vcruise velocity - see documentation */
                climberRightExtendSRX.configMotionCruiseVelocity(1889, Constants.ClimberExtendConstants.kTimeoutMs);
                climberRightExtendSRX.configMotionAcceleration(1889.1, Constants.ClimberExtendConstants.kTimeoutMs);

                climberRightExtendSRX.configFeedbackNotContinuous(true, Constants.ClimberExtendConstants.kTimeoutMs);

                // Configure current limits
                climberRightExtendSRX.configPeakCurrentLimit(30);
                climberRightExtendSRX.configPeakCurrentDuration(150);

                // takes in AMPS
                climberRightExtendSRX.configContinuousCurrentLimit(20);

                // integral zone
                climberRightExtendSRX.config_IntegralZone(Constants.ClimberExtendConstants.kSlotIdx, 3);

                /* Set relevant frame periods to be at least as fast as periodic rate */
                climberLeftExtendSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
                                Constants.ClimberExtendConstants.kTimeoutMs);

                /* Set the peak and nominal outputs */
                climberLeftExtendSRX.configNominalOutputForward(0, Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.configNominalOutputReverse(0, Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.configPeakOutputForward(1, Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.configPeakOutputReverse(-1, Constants.ClimberExtendConstants.kTimeoutMs);

                /* Set Motion Magic gains in slot0 - see documentation */
                climberLeftExtendSRX.selectProfileSlot(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.kPIDLoopIdx);
                climberLeftExtendSRX.config_kF(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kF,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.config_kP(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kP,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.config_kI(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kI,
                                Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.config_kD(Constants.ClimberExtendConstants.kSlotIdx,
                                Constants.ClimberExtendConstants.gains.kD,
                                Constants.ClimberExtendConstants.kTimeoutMs);

                /* Set acceleration and vcruise velocity - see documentation */
                climberLeftExtendSRX.configMotionCruiseVelocity(1889, Constants.ClimberExtendConstants.kTimeoutMs);
                climberLeftExtendSRX.configMotionAcceleration(1889.1, Constants.ClimberExtendConstants.kTimeoutMs);

                climberLeftExtendSRX.configFeedbackNotContinuous(true, Constants.ClimberExtendConstants.kTimeoutMs);

                // Configure current limits
                climberLeftExtendSRX.configPeakCurrentLimit(30);
                climberLeftExtendSRX.configPeakCurrentDuration(150);

                // takes in AMPS
                climberLeftExtendSRX.configContinuousCurrentLimit(20);

                // integral zone
                climberLeftExtendSRX.config_IntegralZone(Constants.ClimberExtendConstants.kSlotIdx, 3);
        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
                SmartDashboard.putNumber("left arm", climberLeftExtendSRX.getSelectedSensorPosition());
                SmartDashboard.putNumber("right arm", climberRightExtendSRX.getSelectedSensorPosition());

                // if (leftSensor.get() == false) {
                // climberLeftExtendSRX.setSelectedSensorPosition(0);
                // }

                // if (rightSensor.get() == false) {

                // climberRightExtendSRX.setSelectedSensorPosition(0);
                // }

                // SmartDashboard.putBoolean("right sensor", rightSensor.get());
                // SmartDashboard.putBoolean("left sensor", leftSensor.get());

                // if (leftStatus && rightStatus) {
                // climberLeftExtendSRX.setSelectedSensorPosition(25);
                // climberRightExtendSRX.setSelectedSensorPosition(25);
                // }
        }

}