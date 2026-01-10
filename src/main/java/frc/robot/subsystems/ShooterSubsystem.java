package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HarwareConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private SparkMax followerMotor;

    private final boolean followerEnabled = false;

    /**Constructor */
    public ShooterSubsystem() {
        motor = new SparkMax(SHOOTER_ID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        if (followerEnabled) {
            followerMotor = new SparkMax(SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
            SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
            followerMotorConfig.follow(SHOOTER_ID);
        }
    }

    /**Command to "set and forget" the motor speed */
    public Command setSpeed(double speed) {
        return runOnce(() -> {
            motor.set(speed);
        });
    }

    /**Command with end statement to set the motor speed to zero */
    public Command runShooter(double speed) {
        return runEnd(() -> {
            motor.set(speed);
        }, ()-> {
            motor.set(0);
        });
    }

    @Override
    public void putDashboard(){
        SmartDashboard.putNumber("Motor speed", this.motor.getEncoder().getVelocity());
    }

    @Override public void periodic(){

    }
}
