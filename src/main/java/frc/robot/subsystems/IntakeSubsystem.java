package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import static frc.robot.Constants.HardwareConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // motor controllers
    private final ThriftyNova extensionMotor;
    private final ThriftyNova extensionMotor2;
    private final ThriftyNova intakeMotor;
    private final ThriftyNova conveyorMotor;

    private final PIDController controller = new PIDController(0.8, 0, 0);
    private double extSetpoint = 0;

    private double setSpeed = 0;

    public IntakeSubsystem() {
        intakeMotor = new ThriftyNova(INTAKE_ID);
        intakeMotor.setInversion(true);
        extensionMotor = new ThriftyNova(EXTENSION_MOTOR_ID);
        extensionMotor2 = new ThriftyNova(EXTENSION_MOTOR2_ID);
        extensionMotor2.setInversion(false).follow(EXTENSION_MOTOR_ID);
        conveyorMotor = new ThriftyNova(CONVEYER_MOTOR_ID);
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

    /** run once command to set the intake motor speed */
    public Command setConveyor(double speed) {
        return runOnce(() -> {
            conveyorMotor.set(speed);
        });
    }

    /** Run end command to run the conveyor motor - upon ending will stop */
    public Command runConveyor(double speed) {
        return runEnd(() -> {
            conveyorMotor.set(speed);
        }, () -> {
            conveyorMotor.set(0);
        });
    }

    /**
     * run end command to run the extension motor - upon ending will stop and enter
     * brake mode
     */
    public Command runExtension(double output) {
        return runEnd(() -> {
            extensionMotor.set(output);
            //extensionMotor2.set(output);
        }, () -> {
            extensionMotor.set(0);
            //extensionMotor2.set(0);
            //extensionMotor.setBrakeMode(true);
           // extensionMotor2.setBrakeMode(true);
        });
    }

    public Command extendIn() {
        return runExtension(IntakeConstants.EXTEND_IN_SPEED)
                .withTimeout(1);
    }

    public Command extendOut() {
        return runExtension(IntakeConstants.EXTEND_OUT_SPEED)
                .withTimeout(1);
    }

    /** Updates the position of the extenstion */
    public Command setExt(double position) {
        return runOnce(() -> {
            extSetpoint = position;
        });
    }

    private double getExtEffort() {
        return controller.calculate(extensionMotor.getPositionInternal(), extSetpoint);
    }

    @Override
    public void periodic() {
        // TODO add
        //extensionMotor.setVoltage(getExtEffort());
        //extensionMotor2.setVoltage(getExtEffort());
        SmartDashboard.putNumber("Extension setpoint", extSetpoint);
        SmartDashboard.putNumber("Extension calculated effort", getExtEffort());
        SmartDashboard.putNumber("intake set speed", setSpeed);
        SmartDashboard.putNumber("extension position", extensionMotor.getPositionInternal());
    }
}
