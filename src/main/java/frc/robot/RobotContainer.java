package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
	// Joysticks
	private final CommandXboxController xbox = new CommandXboxController(0);
	private final XboxController operatorXbox = new XboxController(1);

	// Subsystems/custom class instantiation
	private final Vision vision = new Vision();
	private final SwerveSubsystem swerve = new SwerveSubsystem(xbox, true, vision);
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final ClimberSubsystem climber = new ClimberSubsystem();
	private final Superstructure superstructure = new Superstructure(swerve, intake, climber, shooter, vision);

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configBindings();

		// register named commands here

		// config pathplanner
		swerve.configPathPlanner();
		// add auto chooser to dashboard
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		//driver xbox
		xbox.a().onTrue(swerve.temp());

		//operator logitec
		new JoystickButton(operatorXbox, kA.value).onTrue(null);//just and example <3
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void updates() {
		SmartDashboard.putNumber("Cam Skew", vision.getSkew());
		SmartDashboard.putNumber("Cam Pitch", vision.getPitch());
		SmartDashboard.putNumber("Cam Yaw", vision.getYaw());
		SmartDashboard.putNumber("Cam Best Tag", vision.getBestTagID());
	}
}
