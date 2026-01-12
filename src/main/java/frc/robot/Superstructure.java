package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Superstructure {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final Vision vision;
    private final LEDSubsystem leds;

    /** Constructor */
    public Superstructure(SwerveSubsystem swerve, IntakeSubsystem intake,
            ShooterSubsystem shooter, Vision vision, LEDSubsystem leds) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.vision = vision;
        this.leds = leds;
    }

    /**
     * 
     * @param distance from target along the ground (ignores height) in meters
     * @param theta    is the angle of the shot in degrees above the horizon
     * @param height   from point of release to top of rim in meters
     * @return
     */
    public double calculateV(double distance, double theta, double height) {
        theta = Units.degreesToRadians(theta);
        if (!((distance * Math.tan(theta) - height) > 0)) {
        }
        return (distance / Math.cos(theta)) * Math.sqrt(9.8 / (2 * (distance * Math.tan(theta) - height)));
    }

    public Command setupExtension(double time) {
        return intake.runExtension(0.7).withTimeout(time);
    }

    // autons because pathplanner doesnt work
    public Command leftAuto() {//TODO fix
        return Commands.sequence(
                swerve.autoDrive(0.5,1 , 0).withTimeout(1.5),
                swerve.autoDrive(0, 0, 0.5).withTimeout(0.8),
                swerve.autoDrive(0, 0, 0));
    }

      // autons because pathplanner doesnt work
    public Command rightAuto() {//TODO fix
        return Commands.sequence(
                swerve.autoDrive(-0.5,1 , 0).withTimeout(1.5),
                swerve.autoDrive(0, 0, 0.5).withTimeout(0.8),
                swerve.autoDrive(0, 0, 0));
    }
}
