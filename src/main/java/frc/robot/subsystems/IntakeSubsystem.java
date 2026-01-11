package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.thethriftybot.devices.ThriftyEncoder;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ExtensionConstants.*;
import static frc.robot.Constants.HardwareConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // motor controllers
    private final ThriftyNova extensionMotor;
    public final SparkMax intakeMotor;
    private SparkMax followerMotor;

    private final ThriftyEncoder encoder;
    ThriftyNovaConfig config = new ThriftyNovaConfig();

    Command toPosition;

    private double extSetpoint = MIN_EXTENSION;
    private double setSpeed = 0;
    private final boolean followerEnabled = true;//TODO change based on number of motors

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(INTAKE_ID, MotorType.kBrushless);
        if (followerEnabled) {
            followerMotor = new SparkMax(SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
            SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
            followerMotorConfig.follow(SHOOTER_ID, true);
        }

        // extension configs
        encoder = new ThriftyEncoder(EXTENSION_ENCODER_ID);
        extensionMotor = new ThriftyNova(EXTENSION_MOTOR_ID);
        config.pid0.pid.setPID(EXTENSION_P, EXTENSION_I, EXTENSION_D);
        toPosition = new InstantCommand(() -> config.pid0.pid.calculate(encoder.getPosition(), extSetpoint), this);
        setDefaultCommand(toPosition);
    }

    public Command extend() {
        return runOnce(() -> {
            extSetpoint = MAX_EXTENSION;
        });
    }

    public Command retract() {
        return runOnce(() -> {
            extSetpoint = MIN_EXTENSION;
        });
    }

    /** run once command to set the intake motor speed */
    public Command setIntake(double speed) {
        return runOnce(() -> {
            intakeMotor.set(speed);
        });
    }

    /** Run end command to run the intake motor - upon ending will stop */
    public Command runIntake(double speed) {
        return runEnd(() -> {
            intakeMotor.set(speed);
        }, () -> {
            intakeMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake set speed", setSpeed);
    }
}
