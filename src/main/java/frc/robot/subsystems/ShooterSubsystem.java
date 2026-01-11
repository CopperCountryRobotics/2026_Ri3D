package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private SparkMax followerMotor;
    private final SparkMax hoodMotor;

    private final SparkAbsoluteEncoder hoodEncoder;

    private double setSpeed = 0;
    private double hoodSetpoint = 0;

    private final boolean followerEnabled = false;

    private final PIDController controller = new PIDController(0, 0, 0);//TODO tune
    private final ArmFeedforward ffeController = new ArmFeedforward(4, 0.2, 0);//TODO add kv

    /** Constructor */
    public ShooterSubsystem() {
        motor = new SparkMax(SHOOTER_ID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(SHOOTER_CURRENT_LIMIT, SHOOTER_CURRENT_LIMIT);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if (followerEnabled) {
            followerMotor = new SparkMax(SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
            SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
            followerMotorConfig.smartCurrentLimit(SHOOTER_CURRENT_LIMIT, SHOOTER_CURRENT_LIMIT);
            followerMotorConfig.follow(SHOOTER_ID, true);
            followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }

        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getAbsoluteEncoder();
        SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();
        hoodMotorConfig.smartCurrentLimit(HOOD_CURRENT_LIMIT, HOOD_CURRENT_LIMIT);
        hoodMotorConfig.absoluteEncoder.inverted(false);
        hoodMotorConfig.absoluteEncoder.zeroOffset(HOOD_ENCODER_OFFSET);
        hoodMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Command to "set and forget" the motor speed */
    public Command setSpeed(double speed) {
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

    /** returns the absolute encoder of the hood */
    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    /** Updates the position of the hood */
    public Command setHood(double position) {
        return runOnce(() -> {
            hoodSetpoint = position;
        });
    }

    /** Calculate effort for the hood motor */
    public double getHoodEffort() {
        return ffeController.calculate(Units.rotationsToRadians(hoodEncoder.getPosition()), hoodEncoder.getVelocity())
                + controller.calculate(hoodEncoder.getPosition(), hoodSetpoint);
    }

    @Override
    public void periodic() {
        //update hood motor
        //hoodMotor.setVoltage(getHoodEffort());//TODO add later

        // update dashboard
        SmartDashboard.putNumber("Motor speed", this.motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Set speed", setSpeed);
    }
}
