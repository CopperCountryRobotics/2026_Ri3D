package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HarwareConstants.*;
import static frc.robot.Constants.ExtentionConstants.*;
public class ExtensionSubsystem extends SubsystemBase{
    SparkMax motor;
    //Encoder encoder;
    int min = MIN_EXTENTION;
    int max = MAN_EXTENTION;
    int target = min; 
    //Command toPosition = new InstantCommand(()->motor.set();, this)
    public ExtensionSubsystem(){
        motor = new SparkMax(EXTENDSION_ID,MotorType.kBrushless);
    //    setDefaultCommand(toPosition);
    }
    public void extend(){
        target = max;
    }
    public void retract(){
        target = min;
    }
}
