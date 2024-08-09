// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
	public static final class DriveConstants {
		public static double kMaxDistance = 5;

		// Preset Measurements
		public static final double kGearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
		public static final double kCountsPerMotorShaftRev = 12.0;
		public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
		public static final double kWheelDiameterMeter = 0.06;
		public static final double kMaxVel = 0.75;
		public static final double fieldOrientedLeeway = 0.2;
		public static final double fieldOrientedRotateSpeed = 2;
	}

	public static final class ControllerConstants {
		public static final int kPort = 0;
	}

	public static final class IDs {
		// Drive Subsystem
		public static final int kLeftMotor = 0;
		public static final int kRightMotor = 1;
		public static final int kLeftEncoderA = 4;
		public static final int kLeftEncoderB = 5;
		public static final int kRightEncoderA = 6;
		public static final int kRightEncoderB = 7;

		// Servo
		public static final int kServo = 4;
	}
}
