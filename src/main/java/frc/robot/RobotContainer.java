package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
	// Joysticks, switching to joystick bc Noah took his xbox back :(
	private final CommandJoystick joystick = new CommandJoystick(0);
	// private final CommandXboxController xbox = new CommandXboxController(0);
	// private final XboxController operatorXbox = new XboxController(1);

	// Subsystems/custom class instantiation
	private final Vision vision = new Vision();
	// private final SwerveSubsystem swerve = new SwerveSubsystem(xbox, true,
	// vision);
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final IntakeSubsystem intake = new IntakeSubsystem();

	// private final Superstructure superstructure = new Superstructure(swerve,
	// intake, shooter, vision);

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configBindings();

		// register named commands here

		// config pathplanner
		// swerve.configPathPlanner();]\[]
		// add auto chooser to dashboard
		// autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser = new SendableChooser<>();
		// autoChooser.addOption("Left Auto", superstructure.leftAuto());
		// autoChooser.addOption("Right Auto", superstructure.rightAuto());

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		joystick.button(3).onTrue(shooter.setShooter(0));
		joystick.button(4).onTrue(shooter.setShooter(0.01));
		joystick.button(5).onTrue(shooter.setShooter(0.1));
		joystick.button(6).onTrue(shooter.setShooter(0.7));

		joystick.button(7).onTrue(shooter.setHood(0));
		joystick.button(8).onTrue(shooter.setHood(1));
		joystick.button(9).onTrue(shooter.setHood(2));
		joystick.button(10).onTrue(shooter.setHood(3));

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setup() {
		// intake.setIntake(INTAKE_SPEED);
		// shooter.setGate(GATE_SPEED);
		// shooter.setHood(DEFAULT_HOOD_POSITION);
		// superstructure.setupExtension(2);// TODO tune
		System.out.println("Setup is complete!");
	}

	public void updates() {
		vision.updateDashboard();
	}
}
