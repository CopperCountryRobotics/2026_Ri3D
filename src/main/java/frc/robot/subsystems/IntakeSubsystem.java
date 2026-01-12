package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HardwareConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // motor controllers
    private final ThriftyNova extensionMotor;
    public final ThriftyNova intakeMotor;
    private final ThriftyNova gateMotor;

    private double setSpeed = 0;

    public IntakeSubsystem() {
        // intake configs
        intakeMotor = new ThriftyNova(INTAKE_ID);

        // extension configs
        extensionMotor = new ThriftyNova(EXTENSION_MOTOR_ID);

        // Gate motor configs
        gateMotor = new ThriftyNova(GATE_MOTOR_ID);
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

    /** run once command to set the speed of the gate motor */
    public Command setGate(double speed) {
        return runOnce(() -> {
            gateMotor.set(speed);
        });
    }

    /**
     * run end command to run the extension motor - upon ending will stop and enter
     * brake mode
     */
    public Command runExtension(double output) {
        return runEnd(() -> {
            extensionMotor.set(output);
        }, () -> {
            extensionMotor.set(0);
            extensionMotor.setBrakeMode(true);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake set speed", setSpeed);
    }
}
