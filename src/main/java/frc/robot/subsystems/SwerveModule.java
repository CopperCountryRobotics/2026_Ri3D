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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed, String name) {
        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        moduleName = name;
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        absoluteEncoder.setDutyCycleRange(0.0 / 0.0, 0.0 / 0.0);
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
      }
  
  /** Creates a new SwerveModule. */
  public SwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
