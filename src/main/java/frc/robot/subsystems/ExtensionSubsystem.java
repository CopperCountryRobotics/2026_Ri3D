package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyEncoder;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HarwareConstants.*;
import static frc.robot.Constants.ExtentionConstants.*;

public class ExtensionSubsystem extends SubsystemBase {
    private final ThriftyNova motor;
    private final ThriftyEncoder encoder;
    private double target = MIN_EXTENTION;
    Command toPosition;
    ThriftyNovaConfig config = new ThriftyNovaConfig();

    public ExtensionSubsystem() {
        encoder = new ThriftyEncoder(EXTENSION_ENCODER_ID);
        motor = new ThriftyNova(EXTENSION_MOTOR_ID);
        config.pid0.pid.setPID(EXTENTION_P, EXTENTION_I, EXTENTION_D);
        toPosition = new InstantCommand(() -> config.pid0.pid.calculate(encoder.getPosition(), target), this);
        setDefaultCommand(toPosition);
    }

    public Command extend() {
        return runOnce(() -> {
            target = MAX_EXTENTION;
        });
    }

    public Command retract() {
        return runOnce(() -> {
            target = MIN_EXTENTION;
        });
    }
}
