package frc.robot;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Superstructure {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;
    private final ShooterSubsystem shooter;
    private final Vision vision;

    /**Constructor */
    public Superstructure(SwerveSubsystem swerve, IntakeSubsystem intake, ClimberSubsystem climber, ShooterSubsystem shooter, Vision vision){
        this.swerve = swerve;
        this.intake = intake;
        this.climber = climber;
        this.shooter = shooter;
        this.vision = vision;
    }

    //Land of awesomecommands

//     public double calculateV(double distance, double theta, double height){
//         if(!distance*Math.tan(theta)-height)>0){}
//        return (distance/Math.cos(theta))* Math.sqrt(9.8/(2*(distance*Math.tan(theta)-height)));

//     }
}
