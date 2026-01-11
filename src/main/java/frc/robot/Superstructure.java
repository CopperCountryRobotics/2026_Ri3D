package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Superstructure {
    @SuppressWarnings("unused")
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;
    private final ShooterSubsystem shooter;
    private final Vision vision;

    /** Constructor */
    public Superstructure(SwerveSubsystem swerve, IntakeSubsystem intake, ClimberSubsystem climber,
            ShooterSubsystem shooter, Vision vision) {
        this.swerve = swerve;
        this.intake = intake;
        this.climber = climber;
        this.shooter = shooter;
        this.vision = vision;
    }

    // Land of awesomecommands
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
}
