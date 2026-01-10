package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase{
    public SparkMax motor;
    public boolean enabled = false;
    public Intake(){
        motor = new SparkMax(intakeMotorID, MotorType.kBrushless);
    }
    public void setSpeed(double speed){
        motor.set(intakeReversed?-speed:speed);
    }
    public Command Toggle(){
        return new InstantCommand(()->enabled = !enabled);
    }

}
