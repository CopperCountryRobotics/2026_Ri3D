package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.wpilibj.XboxController.Button.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

	 private final Superstructure superstructure = new Superstructure(swerve,
	intake, shooter, vision);

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configBindings();

		// register named commands here

		// config pathplanner
		// swerve.configPathPlanner();
		// add auto chooser to dashboard
		// autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser = new SendableChooser<>();
		// autoChooser.addOption("Left Auto", superstructure.leftAuto());
		// autoChooser.addOption("Right Auto", superstructure.rightAuto());

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		// driver xbox
		xbox.b().whileTrue(superstructure.shoot()).onFalse(superstructure.stopShoot());
		xbox.x().onTrue(shooter.setHood(5));
		xbox.y().onTrue(shooter.setHood(4));
		xbox.a().onTrue(shooter.setHood(3));
		xbox.rightBumper().onTrue(shooter.setHood(0));
		xbox.povDown().onTrue(shooter.setHood(5.3));
		xbox.povLeft().onTrue(shooter.setHood(7));
		xbox.povRight().onTrue(shooter.setHood(5.8));
		xbox.povUp().onTrue(shooter.setHood(5.5));
		xbox.back().onTrue(shooter.setHood(6));
		xbox.start().onTrue(shooter.zeroHood(0)); //Doesn't even use the variable >:(


		// xbox.b().whileTrue(shooter.setGate(0.5));
		// // xbox.x().whileTrue(swerve.faceAprilTag()
		// // .until(() -> swerve.goalRot != 0 && MathUtil.isNear(swerve.goalRot,
		// swerve.yaw, 5)));
		// // xbox.y().whileTrue(intake.runConveyor(0.5));

		// // operator logitec
		// new JoystickButton(operatorXbox,
		// kA.value).onTrue(intake.setIntake(INTAKE_SPEED));
		// new JoystickButton(operatorXbox,
		// kB.value).onTrue(shooter.setHood(DEFAULT_HOOD_POSITION));
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
		SmartDashboard.putNumber("Cam Pitch", vision.getPitch());
		SmartDashboard.putNumber("Cam Yaw", vision.getYaw());
		SmartDashboard.putNumber("Cam Yaw", vision.getFiducialId());

	}
}
