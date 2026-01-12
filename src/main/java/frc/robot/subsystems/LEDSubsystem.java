package frc.robot.subsystems;

import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.connection.usb.USBPort;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final ConnectorX cX;
    private boolean cxConnected = false;

    private int sequenceID = 0;
    private String[] sequences = {"test1", "test2", "test3", "test4", "test5", "test6", "test7"};

    /** Constructor */
    public LEDSubsystem() {
        cX = new ConnectorX();
        cxConnected = cX.Connect(USBPort.kUSB1);

        cX.leds.SetColor("Matrix", new Color(new Color8Bit("#ffcd00")));
    }

    /** Command to set the color of a zone */
    public Command setColor(Color color, String zoneId) {
        return runOnce(() -> {
            cX.leds.SetColor(zoneId, color);
        });
    }

    /** Command to set a looping sequence of a zone */
    public Command setSequence(String sequenceName, String zoneId) {
        return runOnce(() -> {
            cX.leds.SetImageSequence(sequenceName)
                .ForZone(zoneId)
                .RunOnce(false);
        });
    }

    /** Command to progress through the predefined list of sequences zone */
    public Command nextSequence() {
        return runOnce(() -> {
            setSequence(sequences[sequenceID], "Matrix");
            sequenceID++;
            if (sequenceID >= sequences.length) sequenceID = 0;
        });
    }

    @Override
    public void periodic() {
        // update dashboard
        SmartDashboard.putBoolean("led connected", cxConnected);
    }
}
