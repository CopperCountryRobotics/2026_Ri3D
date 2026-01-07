// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMax turningMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;

  private final PIDController turningPidController;
  private PIDController builtinTurningPidController;

  private final DutyCycleEncoder absoluteEncoder;

  private final double absoluteEncoderOffsetRad;

  private String moduleName;

  // Class constructor where we assign default values for variable
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed, String name) {
        // Set offsets for absolute encoder in RADIANS
        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        moduleName = name;
        // Create absolute encoder
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        // Set duty cycle range of encoder of ABE encoder
        absoluteEncoder.setDutyCycleRange(0.0 / 0.0, 0.0 / 0.0);
        // Create drive and turning motor
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        // Set reverse state of drive and turning motor
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        // Set drive and turning motor encoder values
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        // Change drive motor conversion factors
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // Change conversion factors for neo turning encoder - should be in radians
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        // Create PID controller on ROBO RIO
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // Tell PID controller that it is a *wheel*
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // -----SPARK-MAX-PID-----//
        builtinTurningPidController = turningMotor.getPIDController();
        // Set PID values for the simulated Spark max PID
        builtinTurningPidController.setP(ModuleConstants.kPTurning);
        builtinTurningPidController.setI(ModuleConstants.kITurning);
        builtinTurningPidController.setD(ModuleConstants.kDTurning);
        builtinTurningPidController.setIZone(0.0);
        builtinTurningPidController.setFF(0.0);
        builtinTurningPidController.setOutputRange(-1, 1);
        turningMotor.burnFlash();
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(40);
        turningMotor.setSmartCurrentLimit(20);
        // Call resetEncoders
        resetEncoders();
        // Create the mechanism 2d canvas and get the root
        Mechanism2d mod = new Mechanism2d(6, 6);
        MechanismRoot2d root = mod.getRoot("climber", 3, 3);
      }

  public void update() {
    SmartDashboard.putString(moduleName + " Position", getPosition().toString());
  }

  // Helpful get methods
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
    // return getAbsoluteEncoderRad();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public SwerveModulePosition getPosition() {
    return (new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition())));
  }

  public double getAbsoluteEncoderRad() {

    // Make angle variable
    double angle;

    // Get encoder absolute position goes from 1 to 0
    angle = (1 - absoluteEncoder.getAbsolutePosition()) * 2.0 * Math.PI;

    angle -= absoluteEncoderOffsetRad;

    return angle;
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    REVLibError error = turningEncoder.setPosition(getAbsoluteEncoderRad());
    if (error.value != REVLibError.kOk.value) {
    System.out.println("ENCODER ERROR ON " + error.value + " " + moduleName);
    }
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  
    public void setDesiredState(SwerveModuleState state) {

    // Check if new command has high driving power
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    turningMotor
        .set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    // simTurn.setAngle(state.angle);
    // simDirection.setAngle(state.speedMetersPerSecond > 0 ? 0 : 180);

    // simTurn2.setAngle(absoluteEncoder.getAbsolutePosition());
    // simDirection2.setAngle(state.speedMetersPerSecond /
    // DriveConstants.kPhysicalMaxSpeedMetersPerSecond > 0 ? 0 : 180);
    SmartDashboard.putString(moduleName + " state", state.toString());

  }

  public void setDesiredState(SwerveModuleState state) {
    // Check if new command has high driving power
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);

    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    turningMotor
        .set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString(moduleName + " state", state.toString());

  }

  // Stop all motors on module
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // Motor and SparkMax methods for Monitor
  public double[] getMotorsCurrent() {
    return (new double[] { driveMotor.getOutputCurrent(), turningMotor.getOutputCurrent() });
  }

  public double[] getMotorsTemp() {
    return (new double[] { driveMotor.getMotorTemperature(), turningMotor.getMotorTemperature() });
  }

  public void setSmartCurrentLimiter(int driveLimit, int turningLimit) {
    driveMotor.setSmartCurrentLimit(driveLimit);
    turningMotor.setSmartCurrentLimit(driveLimit);
  }
  
  /** Creates a new SwerveModule. */
  public SwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
