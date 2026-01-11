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

    /**Constructor */
    public Superstructure(SwerveSubsystem swerve, IntakeSubsystem intake, ClimberSubsystem climber, ShooterSubsystem shooter){
        this.swerve = swerve;
        this.intake = intake;
        this.climber = climber;
        this.shooter = shooter;
    }

    //Land of awesomecommands
}
