package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

public class RobotContainer {
	// Joysticks
	private final CommandXboxController xbox = new CommandXboxController(0);
	// private final XboxController operatorXbox = new XboxController(1);
	private final CommandJoystick joystick = new CommandJoystick(1);

	// Subsystems/custom class instantiation
	private final Vision vision = new Vision();
	private final SwerveSubsystem swerve = new SwerveSubsystem(xbox, true, vision);
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final IntakeSubsystem intake = new IntakeSubsystem();

	private final Superstructure superstructure = new Superstructure(swerve,
			intake, shooter, vision);

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configBindings();

		// register named commands here

		// config pathplanner
		swerve.configPathPlanner();
		// add auto chooser to dashboard
		autoChooser = AutoBuilder.buildAutoChooser();
		// autoChooser = new SendableChooser<>();
		// autoChooser.addOption("Left Auto", superstructure.leftAuto());
		// autoChooser.addOption("Right Auto", superstructure.rightAuto());

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		// driver xbox
		xbox.a().whileTrue(superstructure.reverseShooter()).onFalse(superstructure.stopShoot());
		xbox.start().whileTrue(swerve.strafeToTag());
		xbox.b().whileTrue(swerve.faceAprilTag());
		xbox.y().onTrue(superstructure.stopShoot());
		xbox.x().onTrue(superstructure.shoot());

		xbox.povDown().onTrue(intake.setIntake(0));
		xbox.povUp().onTrue(intake.runIntake(IntakeConstants.INTAKE_SPEED));
		xbox.povRight().whileTrue(intake.extendOut());
		xbox.povLeft().whileTrue(intake.extendIn());
		// xbox.povRight().whileTrue(intake.setExt(2));
		// xbox.povLeft().whileTrue(intake.setExt(0));

		// xbox.leftBumper().whileTrue(shooter.runHood(0.07));
		// xbox.rightBumper().whileTrue(shooter.runHood(-0.07));
		xbox.leftBumper().onTrue(shooter.setHood(5));
		xbox.rightBumper().onTrue(shooter.setHood(3));

		xbox.back().onTrue(swerve.resetGyro());

		// // // operator logitec
		// new JoystickButton(operatorXbox,
		// kA.value).onTrue(intake.setIntake(INTAKE_SPEED));
		// // new JoystickButton(operatorXbox,
		// // kB.value).onTrue(shooter.setHood(DEFAULT_HOOD_POSITION));

		joystick.button(3).onTrue(shooter.setHood(0));
		joystick.button(4).onTrue(shooter.setHood(0.5));
		joystick.button(5).onTrue(shooter.setHood(1));
		joystick.button(6).onTrue(shooter.setHood(1.5));
		joystick.button(7).onTrue(superstructure.setupExtension(3,0));
		joystick.button(8).onTrue(intake.setExt(0.5));
		joystick.button(9).onTrue(intake.setExt(1));
		joystick.button(10).onTrue(intake.setExt(3.5));



	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setup() {
		// intake.setIntake(INTAKE_SPEED);
		// shooter.setGate(GATE_SPEED);
		// shooter.setHood(DEFAULT_HOOD_POSITION);
		superstructure.setupExtension(2,0);// TODO tune
		System.out.println("Setup is complete!");
	}

	public void updates() {
		vision.updateDashboard();
	}
}
