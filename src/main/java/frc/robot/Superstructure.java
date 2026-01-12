package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Superstructure {
    @SuppressWarnings("unused")
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final Vision vision;

    /** Constructor */
    public Superstructure(SwerveSubsystem swerve, IntakeSubsystem intake,
            ShooterSubsystem shooter, Vision vision) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.vision = vision;
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

    public Command setupExtension(double time){
        return intake.runExtension(0.7).withTimeout(time);
    }

    // public Commands j(){
    //     return Commands.sequence(
            
    //         ).until(()->true);
    // }


}
