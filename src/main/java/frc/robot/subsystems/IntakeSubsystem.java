package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.devices.ThriftyEncoder;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.HardwareConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // motor controllers
    private final ThriftyNova extensionMotor;
    public final ThriftyNova intakeMotor;
    private ThriftyNova followerMotor;
    private final ThriftyNova gateMotor;

    private final ThriftyEncoder encoder;
    ThriftyNovaConfig config = new ThriftyNovaConfig();

    Command toPosition;

    private double extSetpoint = MIN_EXTENSION;
    private double setSpeed = 0;
    private final boolean followerEnabled = true;// TODO change based on number of motors

    public IntakeSubsystem() {
        // intake configs
        intakeMotor = new ThriftyNova(INTAKE_ID);
        ThriftyNovaConfig motorConfig = new ThriftyNovaConfig();
        intakeMotor.setBrakeMode(false);
        intakeMotor.setMaxCurrent(CurrentType.STATOR, INTAKE_CURRENT_LIMIT);
        intakeMotor.setMaxCurrent(CurrentType.SUPPLY, INTAKE_CURRENT_LIMIT);

        if (followerEnabled) {
            followerMotor = new ThriftyNova(SHOOTER_FOLLOWER_ID);
            ThriftyNovaConfig followerMotorConfig = new ThriftyNovaConfig();
            followerMotor.setBrakeMode(false);
            followerMotor.setMaxCurrent(CurrentType.STATOR, INTAKE_CURRENT_LIMIT);
            followerMotor.setMaxCurrent(CurrentType.SUPPLY, INTAKE_CURRENT_LIMIT);
            followerMotor.follow(INTAKE_ID);
        }

        // extension configs
        encoder = new ThriftyEncoder(EXTENSION_ENCODER_ID);
        extensionMotor = new ThriftyNova(EXTENSION_MOTOR_ID);
        config.pid0.pid.setPID(0, 0, 0);//TODO tune
        toPosition = new InstantCommand(() -> extensionMotor.usePIDSlot(PIDSlot.SLOT0));
        setDefaultCommand(toPosition);

        //Gate motor configs
        gateMotor = new ThriftyNova(GATE_MOTOR_ID);
    }

    /** Command to set the extender out */
    public Command extend() {
        return runOnce(() -> {
            extSetpoint = MAX_EXTENSION;
        });
    }

    /** Command to set the extender in */
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

    /**run once command to set the speed of the gate motor */
    public Command setGate(double speed){
        return runOnce(()->{
            gateMotor.set(speed);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake set speed", setSpeed);
    }
}
