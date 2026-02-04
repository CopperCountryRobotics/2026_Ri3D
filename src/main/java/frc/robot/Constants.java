// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static final class HardwareConstants {

    // Drive Encoder Ports
    public static final int BACK_LEFT_ENCODER_PORT = 0;
    public static final int FRONT_RIGHT_ENCODER_PORT = 1;
    public static final int BACK_RIGHT_ENCODER_PORT = 2;
    public static final int FRONT_LEFT_ENCODER_PORT = 3;

    // Drive CAN ids
    public static final int FRONT_RIGHT_TURN_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_ID = 2;
    public static final int BACK_RIGHT_TURN_ID = 3;
    public static final int BACK_RIGHT_DRIVE_ID = 4;
    public static final int BACK_LEFT_TURN_ID = 5;
    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int FRONT_LEFT_TURN_ID = 7;
    public static final int FRONT_LEFT_DRIVE_ID = 8;
    public static final int GYRO_ID = 9;

    // Shooter CAN ids
    public static final int HOOD_MOTOR_ID = 20;
    public static final int SHOOTER_ID = 11;
    public static final int GATE_MOTOR_ID = 12;
    public static final int HOOD_SWITCH = 6; // DIO INPUT 6

    // Intake CAN ids
    public static final int INTAKE_ID = 13;
    public static final int EXTENSION_MOTOR_ID = 14;
    public static final int CONVEYER_MOTOR_ID = 15;
    public static final int EXTENSION_MOTOR2_ID = 16;

    // Climber CAN ids
    public static final int CLIMBER_ID = 17;

    // Current limits
    public static final int INTAKE_CURRENT_LIMIT = 30;
    public static final int SHOOTER_CURRENT_LIMIT = 30;
    public static final int HOOD_CURRENT_LIMIT = 30;

    // encoder offsets
    public static final double HOOD_ENCODER_OFFSET = 0;// TODO update
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);

    public static final double kDriveMotorGearRatio = 6.75 / 1.0;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;

    public static final double kDriveEncoderRot2Meters = (2.0 * Math.PI) * (kWheelDiameterMeters / 2.0)
        / kDriveMotorGearRatio;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meters / 60.0;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

    public static final double kPTurning = 0.0;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;

    public static final double kDriveEncoderRot2Meter = 1 / (Units.inchesToMeters(4) * Math.PI); // 3.132965055;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(0.0);
    public static final double kWheelBase = Units.inchesToMeters(0.0);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

    // Driving Motor Ports
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kFrontRightDriveMotorPort = 0;
    public static final int kBackRightDriveMotorPort = 0;
    public static final int kBackLeftDriveMotorPort = 0;

    // Turning Motor Ports
    public static final int kFrontLeftTurningMotorPort = 0;
    public static final int kFrontRightTurningMotorPort = 0;
    public static final int kBackRightTurningMotorPort = 0;
    public static final int kBackLeftTurningMotorPort = 0;

    // Encoder for NEO drive
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    // Encoder on NEO turning
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    // DIO ports on the roboRIO - MagEncoders from swerve module.
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
    public static final int kBackRightDriveAbsoluteEncoderPort = 0;

    // Absolute encoders reversed
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    // Ri3D Competition Robot
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0.0);
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math
        .toRadians(0.0);
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math
        .toRadians(0.0);
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math
        .toRadians(0.0);

    public static final double kPhysicalMaxSpeedMetersPerSecond = 0.0;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = Math.toRadians(0.0);

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.0;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.0;

    public static final double kPThetaController = 0.0;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;

    public static final double kMaxDriveMotorTemp = 0.0;
  }

  public static final class SwerveConstants {
    // Take note that the perimeter rules has changed for this season's game
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.75); // Was 23.75 before reducing drivetrain size
    public static final double TRACK_LENGTH = Units.inchesToMeters(21.75); // Was 23.75 before reducing drivetrain size
    public static final double WHEEL_RADIUS = Units.inchesToMeters(4 / 2.0);
    public static final double DRIVE_GEAR_RATIO = 300.0 / 79.0;
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public static final int MAX_DRIVE_VOLTAGE = 12;
    public static final int MAX_TURN_VOLTAGE = 8;

    public static final double MAX_SPEED = Units.feetToMeters(15.1);
    public static final double MAX_ACCELERATION = 10.0;
    public static final double MAX_ANGULAR_ACCELERATION = 10.0;

    public static final double DEAD_BAND = 0.05;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-TRACK_LENGTH / 2.0, -TRACK_LENGTH / 2.0));
  }

  public static final class IntakeConstants {
    public static final double INTAKE_SPEED = 0.5;
    public static final double CONVEYER_SPEED = 0.7;
    public static final double EXTEND_IN_SPEED = 0.3;
    public static final double EXTEND_OUT_SPEED = -0.3;
    public static final double EXTEND_OUT_POSITION = 4;
    public static final double EXTEND_IN_POSITION = 0;
  }

  public static final class ShooterConstants {
    public static final double DEFAULT_HOOD_POSITION = 0.2;
    public static final double SHOOTER_SPEED = 0.8;
    public static final double GATE_SPEED = 0.6;
  }
}
