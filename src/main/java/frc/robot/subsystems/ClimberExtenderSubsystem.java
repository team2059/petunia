// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberExtenderSubsystem extends SubsystemBase {

        private static CANSparkMax leftNeo = new CANSparkMax(Constants.ClimberExtendConstants.climberLeftExtendNeo,
                        MotorType.kBrushless);
        private SparkMaxPIDController leftPidController = leftNeo.getPIDController();
        private static RelativeEncoder leftEncoder = leftNeo.getEncoder();

        private static CANSparkMax rightNeo = new CANSparkMax(Constants.ClimberExtendConstants.climberRightExtendNeo,
                        MotorType.kBrushless);
        private SparkMaxPIDController rightPidController = rightNeo.getPIDController();
        private static RelativeEncoder rightEncoder = rightNeo.getEncoder();

        public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
        // initialze PID controller and encoder objects

        public static CANSparkMax getLeftMotor() {
                return leftNeo;
        }

        public static CANSparkMax getRightMotor() {
                return rightNeo;
        }

        public static void stopMotors() {
                rightNeo.set(0);
                leftNeo.set(0);
        }

        public static void reset() {
                rightEncoder.setPosition(0);
                leftEncoder.setPosition(0);
        }

        public static void isAtSamePos(double leftPos, double rightPos) {
                while (leftPos < rightPos) {
                        rightNeo.set(0);
                }
                while (leftPos > rightPos) {
                        leftNeo.set(0);
                }
        }

        /** Creates a new Drive. */
        public ClimberExtenderSubsystem() {

                leftNeo.restoreFactoryDefaults();
                leftNeo.setInverted(true);
                // leftEncoder.setInverted(true);

                leftEncoder.setPosition(0);
                leftNeo.setIdleMode(IdleMode.kBrake);

                rightNeo.restoreFactoryDefaults();

                rightEncoder.setPosition(0);
                rightNeo.setIdleMode(IdleMode.kBrake);

                // PID coefficients
                kP = 5e-5;
                kI = 1e-6;
                kD = 0;
                kIz = 0;
                kFF = 0.000156;
                kMaxOutput = 1;
                kMinOutput = -1;
                maxRPM = 5700;

                // Smart Motion Coefficients
                maxVel = 5700; // rpm
                maxAcc = 5700;
                allowedErr = 0.0;

                leftPidController.setP(kP);
                leftPidController.setI(kI);
                leftPidController.setD(kD);
                leftPidController.setIZone(kIz);
                leftPidController.setFF(kFF);
                leftPidController.setOutputRange(kMinOutput, kMaxOutput);

                rightPidController.setP(kP);
                rightPidController.setI(kI);
                rightPidController.setD(kD);
                rightPidController.setIZone(kIz);
                rightPidController.setFF(kFF);
                rightPidController.setOutputRange(kMinOutput, kMaxOutput);

                int smartMotionSlot = 0;
                leftPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
                leftPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
                leftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
                leftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

                rightPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
                rightPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
                rightPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
                rightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        }

        @Override
        public void periodic() {
             //   SmartDashboard.putNumber("left extend", leftEncoder.getPosition());
               // SmartDashboard.putNumber("right extend", rightEncoder.getPosition());
        }
}
