package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.GATE_MOTOR_ID;
import static frc.robot.Constants.HardwareConstants.HOOD_MOTOR_ID;
import static frc.robot.Constants.HardwareConstants.HOOD_SWITCH;
import static frc.robot.Constants.HardwareConstants.SHOOTER_ID;

public class ShooterSubsystem extends SubsystemBase {
    private final ThriftyNova shooterMotor;
    private final ThriftyNova hoodMotor;
    private final ThriftyNova gateMotor;
    private final DigitalInput hoodSwitch;

    private double liveHoodOffset = 0;

    private double setSpeed = 0;

    /** Constructor */
    public ShooterSubsystem() {
        shooterMotor = new ThriftyNova(SHOOTER_ID, MotorType.NEO);
        shooterMotor.pid0.setP(0.7);
        shooterMotor.pid0.setI(0.0001);
        shooterMotor.pid0.setD(0.0);
        shooterMotor.pid0.setFF(8.0); // Is probably a little under the nominal amount
        shooterMotor.pid0.setAccumulatorCap(0.05);

        hoodMotor = new ThriftyNova(HOOD_MOTOR_ID, MotorType.NEO);
        hoodMotor.useEncoderType(EncoderType.INTERNAL);
        hoodMotor.setEncoderPosition(0);
        hoodMotor.pid0.setP(0.0);// 004);//TODO add back
        hoodMotor.pid0.setI(0.0000);
        hoodMotor.pid0.setD(0.0);
        hoodMotor.pid0.setFF(0.0);// 0.7);//TODO addback
        hoodMotor.pid0.setAccumulatorCap(0.00005);
        hoodMotor.pid0.setAllowableError(0.1);

        hoodSwitch = new DigitalInput(HOOD_SWITCH);

        gateMotor = new ThriftyNova(GATE_MOTOR_ID, MotorType.NEO);
    }

    /** Command to "set and forget" the shooter motor speed */
    public Command setShooter(double speed) {
        return runOnce(() -> {
            shooterMotor.setVelocity(speed);
            setSpeed = speed;
        });
    }

    /** Command with end statement to set the motor speed to zero */
    public Command runShooter(double speed) {
        return runEnd(() -> {
            shooterMotor.setPercent(speed);
        }, () -> {
            shooterMotor.setPercent(0);
        });
    }

    /** returns the shooter motors speed in revolutions per second */
    public double getShooterSpeed() {
        return shooterMotor.getVelocity();
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

    /** zeros the position of the hood */
    public Command zeroHood() {
        return runOnce(() -> {
            hoodMotor.setMaxCurrent(CurrentType.SUPPLY, 2); // Super low current limit to protect pulley
            hoodMotor.set(-0.1); // TODO Update to appropriate speed
            Commands.waitUntil(() -> hoodSwitch.get());
            hoodMotor.setEncoderPosition(0);
            hoodMotor.setPosition(0);
        });
    }

    // **just adds a live offset to our encoder position */
    public Command zeroHood2() {
        return runOnce(() -> {
            liveHoodOffset = hoodMotor.getPosition();
        });
    }

    /** Updates the position of the hood */
    public Command setHood(double position) {
        return runOnce(() -> {
            // hoodSetpoint = position;//
            hoodMotor.setPosition(position - liveHoodOffset);
        });
    }

    /**
     * run end command to move the hood motor at a percent speed, and to zero upon
     * ending
     */
    public Command runHood(double speed) {
        return runEnd(() -> {
            hoodMotor.set(speed);
        }, () -> {
            hoodMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        // update dashboard
        SmartDashboard.putNumber("Motor speed", this.shooterMotor.getVelocity());
        SmartDashboard.putNumber("Set speed", setSpeed);
        SmartDashboard.putNumber("Hood encoder", hoodMotor.getPositionInternal());
        SmartDashboard.putNumber("Hood encoder rotations?", hoodMotor.getPositionInternal()/(4096/25));
        SmartDashboard.putNumber("Hood set point", hoodMotor.getSetPoint());

    }
}
