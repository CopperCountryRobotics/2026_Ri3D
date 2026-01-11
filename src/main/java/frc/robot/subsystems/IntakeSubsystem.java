package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.HarwareConstants.*;

public class IntakeSubsystem extends SubsystemBase{
    public final SparkMax motor;
    private SparkMax followerMotor;
    public boolean enabled = false;
    private boolean reversed = false;
    public boolean followerEnabled = true;

    public IntakeSubsystem(){
        motor = new SparkMax(INTAKE_ID, MotorType.kBrushless);
        if (followerEnabled) {
            followerMotor = new SparkMax(SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
            SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
            followerMotorConfig.follow(SHOOTER_ID,true);
        }
    }

    public void setSpeed(double speed){
        motor.set(reversed?-speed:speed);
    }

    public void enable(){
        enabled = true;
        setSpeed(intakeSpeed);
    }

    public void disable(){
        enabled = true;
        setSpeed(0);
    }

    public Command toggle(){
        if (enabled) {
            return new InstantCommand(()->disable(), this);
        }else{
            return new InstantCommand(()->enable(), this);
        }
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("intake enabled", enabled);
    }
}
