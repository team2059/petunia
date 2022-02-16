// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberAngleSubsystem extends SubsystemBase {
  /** Creates a new ClimberAngleSubsystem. */

  static WPI_TalonSRX climberAngleTalonSRX = new WPI_TalonSRX(ClimberConstants.climberAngleTalonSRX);

  public static WPI_TalonSRX getClimberAngleTalonSRX() {
    return climberAngleTalonSRX;
  }

  public ClimberAngleSubsystem() {

    /* Factory default hardware to prevent unexpected behavior */
		climberAngleTalonSRX.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		climberAngleTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ClimberConstants.kPIDLoopIdx,
				ClimberConstants.kTimeoutMs);

    
		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		climberAngleTalonSRX.configNeutralDeadband(0.001, ClimberConstants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		climberAngleTalonSRX.setSensorPhase(false);
		climberAngleTalonSRX.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		climberAngleTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ClimberConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		climberAngleTalonSRX.configNominalOutputForward(0, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.configNominalOutputReverse(0, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.configPeakOutputForward(1, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.configPeakOutputReverse(-1, ClimberConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
    /*TODO TUNE PID VALUES*/
		climberAngleTalonSRX.selectProfileSlot(ClimberConstants.kSlotIdx, ClimberConstants.kPIDLoopIdx);
		climberAngleTalonSRX.config_kF(ClimberConstants.kSlotIdx, ClimberConstants.kAngleGains.kF, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.config_kP(ClimberConstants.kSlotIdx, ClimberConstants.kAngleGains.kP, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.config_kI(ClimberConstants.kSlotIdx, ClimberConstants.kAngleGains.kI, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.config_kD(ClimberConstants.kSlotIdx, ClimberConstants.kAngleGains.kD, ClimberConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
    //TUNE CRUISE VELOCITY WHEN TESTING
		climberAngleTalonSRX.configMotionCruiseVelocity(1000, ClimberConstants.kTimeoutMs);
		climberAngleTalonSRX.configMotionAcceleration(1000, ClimberConstants.kTimeoutMs);

    climberAngleTalonSRX.configFeedbackNotContinuous(true, ClimberConstants.kTimeoutMs);

    //configure current limits
    climberAngleTalonSRX.configPeakCurrentLimit(30);
    climberAngleTalonSRX.configPeakCurrentDuration(150);

    //takes in amps
    climberAngleTalonSRX.configContinuousCurrentLimit(20);

    //set integral zone
    climberAngleTalonSRX.config_IntegralZone(ClimberConstants.kSlotIdx, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle Encoder Val", climberAngleTalonSRX.getSelectedSensorPosition());
  }
}
