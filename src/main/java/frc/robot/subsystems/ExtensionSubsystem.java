package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static 

public class ExtensionSubsystem extends SubsystemBase{
    SparkMax motor;
    //Encoder encoder;
    PIDController pid = new PIDController(kp, ki, kd);
    int min = 0;
    int max;
    public void ExtensionSubsystem(){
        motor = new SparkMax(14);
    }
    public extend(){

    }
    public retract(){

    }
}
