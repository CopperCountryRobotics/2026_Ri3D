package frc.robot.subsystems;

import com.ctre.phoenix6.signals.PIDOutput_PIDOutputModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.devices.ThriftyEncoder;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HarwareConstants.*;
import static frc.robot.Constants.ExtentionConstants.*;
public class ExtensionSubsystem extends SubsystemBase{
    ThriftyNova motor;
    ThriftyEncoder encoder;
    int target = MIN_EXTENTION; 
    Command toPosition;
    ThriftyNovaConfig config= new ThriftyNovaConfig();
    public ExtensionSubsystem(){
        encoder = new ThriftyEncoder(EXTENDSION_ENCODER_ID);
        motor = new ThriftyNova(EXTENDSION_Motor_ID);
        config.pid0.pid.setPID(EXTENTION_P,EXTENTION_I,EXTENTION_D);
        toPosition = new InstantCommand(()->config.pid0.pid.calculate(encoder.getPosition(),target),this);
        setDefaultCommand(toPosition);
    }
    public void extend(){
        target = MAX_EXTENTION;
    }
    public void retract(){
        target = MIN_EXTENTION;
    }
}
