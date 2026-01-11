package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
   // private final SparkMax motor = new SparkMax(CLIMBER_ID, MotorType.kBrushless);

    /**Constructor */
    public ClimberSubsystem(){}


    @Override
    public void periodic(){
       // SmartDashboard.putNumber("Elevator heiht", 0)
    }
}
