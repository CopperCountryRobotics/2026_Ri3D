package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private SparkMax followerMotor;

    private double setSpeed;

    private final boolean followerEnabled = false;

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

    @Override
    public void periodic() {
        // update dashboard
        SmartDashboard.putNumber("Motor speed", this.motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Set speed", setSpeed);
    }
}
