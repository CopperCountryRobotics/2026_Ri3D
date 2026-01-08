package frc.robot.lib.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveEncoder{
    AnalogEncoder coder;
    public SwerveEncoder(int id) {
        coder = new AnalogEncoder(id);
        // config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        // config.MagnetSensor.MagnetOffset = 0.0;
    }

    public Rotation2d getRotation() {
        double value = coder.get();
        return Rotation2d.fromRotations(value > 0.5 ? value - 1.0 : value);
    }
}
