package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HarwareConstants.*;
import static frc.robot.Constants.ExtentionConstants.*;
public class ExtensionSubsystem extends SubsystemBase{
    SparkMax motor;
    //Encoder encoder;
    PIDController pid = new PIDController(kp, ki, kd);
    int min = 0;
    int max;
    public ExtensionSubsystem(){
        motor = new SparkMax(EXTENDSION_ID,MotorType.kBrushless);
    }
    public void extend(){

    }
    public void retract(){

    }
}
