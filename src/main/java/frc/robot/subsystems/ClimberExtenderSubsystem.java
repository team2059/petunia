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

public class ClimberExtenderSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  static WPI_TalonSRX climberExtenderTalonSRX = new WPI_TalonSRX(ClimberConstants.climberExtenderTalonSRX);

  public static WPI_TalonSRX getClimberExtenderTalonSRX() {
    return climberExtenderTalonSRX;
  }

  public ClimberExtenderSubsystem() {

    /* Factory default hardware to prevent unexpected behavior */
		climberExtenderTalonSRX.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		climberExtenderTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ClimberConstants.kPIDLoopIdx,
				ClimberConstants.kTimeoutMs);

    
		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		climberExtenderTalonSRX.configNeutralDeadband(0.001, ClimberConstants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		climberExtenderTalonSRX.setSensorPhase(false);
		climberExtenderTalonSRX.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		climberExtenderTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ClimberConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		climberExtenderTalonSRX.configNominalOutputForward(0, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.configNominalOutputReverse(0, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.configPeakOutputForward(1, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.configPeakOutputReverse(-1, ClimberConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
    /*TODO TUNE PID VALUES*/
		climberExtenderTalonSRX.selectProfileSlot(ClimberConstants.kSlotIdx, ClimberConstants.kPIDLoopIdx);
		climberExtenderTalonSRX.config_kF(ClimberConstants.kSlotIdx, ClimberConstants.kExtenderGains.kF, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.config_kP(ClimberConstants.kSlotIdx, ClimberConstants.kExtenderGains.kP, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.config_kI(ClimberConstants.kSlotIdx, ClimberConstants.kExtenderGains.kI, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.config_kD(ClimberConstants.kSlotIdx, ClimberConstants.kExtenderGains.kD, ClimberConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
    //TUNE CRUISE VELOCITY WHEN TESTING
		climberExtenderTalonSRX.configMotionCruiseVelocity(1000, ClimberConstants.kTimeoutMs);
		climberExtenderTalonSRX.configMotionAcceleration(1000, ClimberConstants.kTimeoutMs);

    climberExtenderTalonSRX.configFeedbackNotContinuous(true, ClimberConstants.kTimeoutMs);

    //configure current limits
    climberExtenderTalonSRX.configPeakCurrentLimit(30);
    climberExtenderTalonSRX.configPeakCurrentDuration(150);

    //takes in amps
    climberExtenderTalonSRX.configContinuousCurrentLimit(20);

    //set integral zone
    climberExtenderTalonSRX.config_IntegralZone(ClimberConstants.kSlotIdx, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Encoder Val", climberExtenderTalonSRX.getSelectedSensorPosition());
  }

  public double getExtenderMotorLocation() {
    return climberExtenderTalonSRX.getSelectedSensorPosition();
  }

  public double getExtenderMotorVelocity() {
    return climberExtenderTalonSRX.getSelectedSensorVelocity();
  }
/* 
  public double getAngleMotorLocation() {
    return angleTalonSRX.getSelectedSensorPosition();
  }

  public double getAngleMotorVelocity() {
    return angleTalonSRX.getSelectedSensorVelocity();
  } */
/* 
  public void setExtenderMotorSpeed(double speed) {
    extenderTalonSRX.set(speed);
  }

  public void setAngleMotorSpeed(double speed) {
    angleTalonSRX.set(speed);
  }  */
}
