package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class FlyweelShooter {
    SparkMax motor;
    SparkMax reverseMotor;
    public FlyweelShooter(){
        motor = new SparkMax(ShooterConstants.ShooterID, MotorType.kBrushless);
        if(ShooterConstants.bothSided)
            reverseMotor = new SparkMax(ShooterConstants.ReverseMotorID, MotorType.kBrushless);
    }
    public void setSpeed(double speed){
        motor.set(ShooterConstants.MotorReversed?-speed:speed);
        if(ShooterConstants.bothSided)
            reverseMotor.set(ShooterConstants.reverseMotorReversed?speed:-speed);
    }

}
