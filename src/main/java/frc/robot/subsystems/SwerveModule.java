// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMax turningMotor;
  private SparkMaxConfig driveMotorConfig;
  private SparkMaxConfig turningMotorConfig;

  private AbsoluteEncoder driveEncoder; //TODO Update
  private AbsoluteEncoder turningEncoder; //TODO Update

  private AbsoluteEncoderConfig driveEncoderConfig; //TODO Update
  private AbsoluteEncoderConfig turningEncoderConfig; //TODO Update

  private final PIDController turningPidController;
  private SparkClosedLoopController builtinTurningPidController;

  private final DutyCycleEncoder absoluteEncoder; //TODO Update

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
        driveMotorConfig.inverted(driveMotorReversed);
        turningMotorConfig.inverted(turningMotorReversed);
        // Set drive and turning motor encoder values
        driveEncoder = driveMotor.getAbsoluteEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();
        // Change drive motor conversion factors
        driveEncoderConfig.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter); //find this i guess
        driveEncoderConfig.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // Change conversion factors for neo turning encoder - should be in radians
        turningEncoderConfig.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoderConfig.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        // Create PID controller on ROBO RIO
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // Tell PID controller that it is a *wheel*
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // -----SPARK-MAX-PID-----//
        builtinTurningPidController = turningMotor.getClosedLoopController();
        // Set PID values for the simulated Spark max PID
        turningMotorConfig.closedLoop.p(ModuleConstants.kPTurning);
        turningMotorConfig.closedLoop.i(ModuleConstants.kITurning);
        turningMotorConfig.closedLoop.d(ModuleConstants.kDTurning);
        turningMotorConfig.closedLoop.iZone(0.0);
        turningMotorConfig.closedLoop.velocityFF(0.0);
        turningMotorConfig.closedLoop.outputRange(-1, 1);
        //turningMotorConfig.burnFlash();

        driveMotorConfig.idleMode(IdleMode.kBrake);
        turningMotorConfig.idleMode(IdleMode.kBrake);
        driveMotorConfig.smartCurrentLimit(40);
        turningMotorConfig.smartCurrentLimit(20);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //turningEncoder.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    angle = (1 - absoluteEncoder.get()) * 2.0 * Math.PI;

    angle -= absoluteEncoderOffsetRad;

    return angle;
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders() {
    //driveEncoder.setPosition(0); //TODO fix
    //REVLibError error = turningEncoder.setPosition(getAbsoluteEncoderRad()); //TODO Fix
    // if (error.value != REVLibError.kOk.value) {
    // System.out.println("ENCODER ERROR ON " + error.value + " " + moduleName);
    //}
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  
  //   public void setDesiredState(SwerveModuleState state) {

  //   // Check if new command has high driving power
  //   if (Math.abs(state.speedMetersPerSecond) < 0.001) {
  //     stop();
  //     return;
  //   }

  //   state = SwerveModuleState.optimize(state, getState().angle);

  //   driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

  //   turningMotor
  //       .set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

  //   // simTurn.setAngle(state.angle);
  //   // simDirection.setAngle(state.speedMetersPerSecond > 0 ? 0 : 180);

  //   // simTurn2.setAngle(absoluteEncoder.getAbsolutePosition());
  //   // simDirection2.setAngle(state.speedMetersPerSecond /
  //   // DriveConstants.kPhysicalMaxSpeedMetersPerSecond > 0 ? 0 : 180);
  //   SmartDashboard.putString(moduleName + " state", state.toString());

  // }

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
    driveMotorConfig.smartCurrentLimit(driveLimit);
    turningMotorConfig.smartCurrentLimit(driveLimit);
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  /** Creates a new SwerveModule. */
  //public SwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
