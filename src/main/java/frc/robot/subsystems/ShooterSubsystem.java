package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final ThriftyNova motor;
    private final ThriftyNova hoodMotor;
    private final ThriftyNova gateMotor;

    private double setSpeed = 0;
    private double hoodSetpoint = 0;

    private final PIDController controller = new PIDController(0, 0, 0);// TODO tune
    private final ArmFeedforward ffeController = new ArmFeedforward(4, 0.2, 0);// TODO add kv

    /** Constructor */
    public ShooterSubsystem() {
        motor = new ThriftyNova(SHOOTER_ID, MotorType.NEO);
        hoodMotor = new ThriftyNova(HOOD_MOTOR_ID, MotorType.NEO);
        gateMotor = new ThriftyNova(GATE_MOTOR_ID);
    }

    /** Command to "set and forget" the shooter motor speed */
    public Command setShooter(double speed) {
        return runOnce(() -> {
            motor.set(speed);
            setSpeed = speed;
        });
    }

    /** Command with end statement to set the motor speed to zero */
    public Command runShooter(double speed) {
        return runEnd(() -> {
            motor.set(speed);
        }, () -> {
            motor.set(0);
        });
    }
    
    /** run once command to set the speed of the gate motor */
    public Command setGate(double speed) {
        return runOnce(() -> {
            gateMotor.set(speed);
        });
    }

    /** returns the position of the hood in degrees */
    public double getHoodPosition() {
        return hoodMotor.getPosition() * 360;
    }

    /** Updates the position of the hood */
    public Command setHood(double position) {
        return runOnce(() -> {
            hoodSetpoint = position;
        });
    }

    /** Calculate effort for the hood motor */
    public double getHoodEffort() {
        return ffeController.calculate(Units.rotationsToRadians(hoodMotor.getPosition()),
                hoodMotor.getVelocity())
                + controller.calculate(hoodMotor.getPosition(), hoodSetpoint);
    }

    public Command testHoodMotor(double speed) {
        return runEnd(() -> {
            hoodMotor.set(speed);
        }, () -> {
            hoodMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        // update hood motor
        // hoodMotor.setVoltage(getHoodEffort());//TODO add later

        // update dashboard
        SmartDashboard.putNumber("Motor speed", this.motor.getVelocity());
        SmartDashboard.putNumber("Set speed", setSpeed);
        SmartDashboard.putNumber("Hood setpoint", hoodSetpoint);
        SmartDashboard.putNumber("Hood encoder", hoodMotor.getPosition());
    }
}
