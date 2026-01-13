package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.wpilibj.XboxController.Button.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

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

	//private final ClimberSubsystem climber = new ClimberSubsystem(); //add if need be
	private final LEDSubsystem leds = new LEDSubsystem();
	private final Superstructure superstructure = new Superstructure(swerve, intake, shooter, vision, leds);

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configBindings();

		// register named commands here

		// config pathplanner
		//swerve.configPathPlanner();
		// add auto chooser to dashboard
		//autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser = new SendableChooser<>();
		autoChooser.addOption("Left Auto", superstructure.leftAuto());
		autoChooser.addOption("Right Auto", superstructure.rightAuto());

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		// driver xbox
		// xbox.a().whileTrue(intake.runExtension(0.5));
		// xbox.b().whileTrue(shooter.setGate(0.5));
		// xbox.x().whileTrue(shooter.runShooter(0.5));
		// xbox.y().whileTrue(shooter.testHoodMotor(0.5));

		// operator logitec
		new JoystickButton(operatorXbox, kA.value).onTrue(intake.setIntake(INTAKE_SPEED));
		new JoystickButton(operatorXbox, kB.value).onTrue(shooter.setHood(DEFAULT_HOOD_POSITION));

		// testing ledsubsystem
		new JoystickButton(operatorXbox, kRightBumper.value).onTrue(leds.setColor(new Color(new Color8Bit("#ffffff")), "Matrix"));
		new JoystickButton(operatorXbox, kStart.value).onTrue(leds.setColor(new Color(new Color8Bit("#00ff00")), "Matrix"));
		new JoystickButton(operatorXbox, kLeftBumper.value).onTrue(leds.nextSequence());
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setup() {
		intake.setIntake(INTAKE_SPEED);
		shooter.setGate(GATE_SPEED);
		shooter.setHood(DEFAULT_HOOD_POSITION);
		superstructure.setupExtension(2);// TODO tune
		System.out.println("Setup is complete!");
	}

	public void updates() {
		SmartDashboard.putNumber("Cam Skew", vision.getSkew());
		SmartDashboard.putNumber("Cam Pitch", vision.getPitch());
		SmartDashboard.putNumber("Cam Yaw", vision.getYaw());
		SmartDashboard.putNumber("Cam Best Tag", vision.getBestTagID());
		vision.updateReadings();
	}
}
