package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.connection.uart.UARTPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final ConnectorX cX;
    private boolean cxConnected = false;

    /** Constructor */
    public LEDSubsystem() {
        cX = new ConnectorX();
        cxConnected = cX.Connect(USBPort.kUSB1);

        cX.leds.SetColor("Matrix", new Color(new Color8Bit("#ffcd00")));
    }

    @Override
    public void periodic() {
        // update dashboard
        SmartDashboard.putBoolean("led connected", cxConnected);
    }
}
