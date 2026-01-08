package frc.robot.lib.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveEncoder{
    AnalogEncoder coder;
    double offsets[] = {306, 233.7, 132.1, 303};
    double offset;
    public SwerveEncoder(int id) {
        coder = new AnalogEncoder(id);
        offset = offsets[id];
        // config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        // config.MagnetSensor.MagnetOffset = 0.0;
    }

    public Rotation2d getRotation() {
        double value = (coder.get() * 360) - offset;
        return Rotation2d.fromDegrees(value > 180 ? value - 360 : value);
    }
}
