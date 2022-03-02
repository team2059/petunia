// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // for paths that are coded. in pathweaver we can set these values
        public static final double kMaxSpeedMetersPerSecond = 0.25;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;

    }

    public static final class DriveConstants {
        // CAN ID Ports
        public static int leftFrontCANSparkMaxCANId = 1;
        public static int leftbackCANSparkMaxCANId = 2;
        public static int rightFrontCANSparkMaxCANId = 3;
        public static int rightBackCANSparkMaxCANId = 4;

        //
        // ratio*2*pi*Units.inchesToMeters(wheel raidus)

        // public static final double kPetuniaGearRatio = 10.71;

        public static final double kPetuniaGearRatio = 12.755;
        public static final double kPetuniaWheelRadiusInches = 3;

        // If we get the number of ticks, say 10000, we want to multiply that times
        // 1/kLinearDistancePerMotorRotation to get number of inches, not 10000 times
        // kLinearDistancePerMotorRotation
        public static final double kLinearDistancePerMotorRotation = (Units
                .inchesToMeters(1 / (kPetuniaGearRatio * 2 * Math.PI
                        * Units.inchesToMeters(kPetuniaWheelRadiusInches)) * 10));

        public static final double ksVolts = 0.1219;
        public static final double kvVoltSecondsPerMeter = 3.343;
        public static final double kaVoltSecondsSquaredPerMeter = 1.0356;
        public static final double kPDriveVel = 2.2662;

        // 21.75 inches equals 0.57785. trackwidth is horizontal distance between the
        // wheels
        public static final double kTrackwidthMeters = Units.inchesToMeters(22.25);
        // DifferentialDriveKinematics allows us to use the trackwidth to convert from
        // chassis speeds to wheel speeds. As elsewhere, we keep our units in meters.
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
    }

    public static final class CollecterConstants {
        // 7:1 bag
        public static final int ballCollecterArmTalonSRX = 5; // should be 5
        public static final int collectorMotorTalonSRX = 6; // should be 6
    }

    public static final class ClimberTiltConstants {
        public static final int climberLeftTiltSRX = 11;
        public static final int climberRightTiltSRX = 12;

        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static final Gains gains = new Gains(5, 0.001, 50, 18.26785714, 0, 0.0);

    }

    public static final class ClimberExtendConstants {
        // 100:1 775pro
        public static final int climberLeftExtendSRX = 9;
        public static final int climberRightExtendSRX = 10;

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 0;

        /**
         * Gains used in Motion Magic, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final Gains gains = new Gains(5, 0.001, 50, 0.4873749404, 0, 0.0);

    }

    public static final class ShooterConstants {
        // 40:1 bag
        public static final int intakeIndexerTalonSRX = 7;
        // 40:1 775pro
        public static final int shooterMotorTalonSRX = 8;

        // Characterization stuff
        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        // Base PID settings
        public static final double kBallShooterkF = 0;
        public static final double kBallShooterkP = 0;
        public static final double kBallShooterkI = 0;
        public static final double kBallShooterkD = 0;
        // These are for different PID configurations --> Set in Motion Magic
        // kSlotIdx sets the PID profile that the motor will pull from (ranges from
        // 1,2,3)
        public static final int kSlotIdx = 0;
        // kPIDLoopIdx sets profile for either Cascading PID loops (1) or non-Cascading
        // PID loops (0)
        public static final int kPIDLoopIdx = 0;

        public static final int kCtreTimeoutMs = 30;
    }

    public static final class CollecterArmConstants {
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 0;

        /**
         * Gains used in Motion Magic, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final Gains kGains = new Gains(2.5, 0.001, 31.4, 3.147692308, 0, 0.0);
    }

    public static final class LimeLightConstants {
        public static final double kLimeLightHeight = 0.0;
        public static final double kLimeLightAngle = 0;
        public static final double kTargetHeight = 0;

        public static final double kTurnP = 0.05; //TODO tune P gains
        public static final double kDistanceP = 0.0;
    }
}
