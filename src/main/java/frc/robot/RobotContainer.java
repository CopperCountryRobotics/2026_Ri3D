package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	// Joysticks
	private final CommandXboxController xbox = new CommandXboxController(0);
	// private final XboxController operatorXbox = new XboxController(1);
	private final CommandJoystick joystick = new CommandJoystick(1);

	// Subsystems/custom class instantiation
	private final Vision vision = new Vision();
	private final SwerveSubsystem swerve = new SwerveSubsystem(xbox, true, vision);
	private final ShooterSubsystem shooter = new ShooterSubsystem(vision, xbox);
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
		NamedCommands.registerCommand("setupExt1", superstructure.autoSetupExtension1());
		NamedCommands.registerCommand("setupExt2", superstructure.autoSetupExtension2());
		NamedCommands.registerCommand("intake", superstructure.intake());


		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {//TODO clean up and rewrite
		//*****GOOD BINDINGS, ADD BACK IN */
		// driver xbox
		// xbox.a().whileTrue(superstructure.reverseShooter()).onFalse(superstructure.stopShoot());
		// xbox.start().whileTrue(swerve.strafeToTag());
		// xbox.b().whileTrue(swerve.faceAprilTag());
		xbox.y().onTrue(superstructure.stopShoot());
		xbox.x().onTrue(superstructure.shoot());
		//xbox.x().whileTrue(shooter.runShooter(0.4));//holdShooter(ShooterConstants.SHOOTER_SPEED));

		xbox.a().onTrue(shooter.bumpHoodDown());
		xbox.b().onTrue(shooter.bumpHoodUp());

		// xbox.y().onTrue(shooter.setHood(3));
		// xbox.x().onTrue(shooter.setHood(4));


		/***GOOD BINDINGS, ADD BACK IN */
		xbox.povDown().onTrue(intake.setIntake(0));
		xbox.povUp().onTrue(intake.runIntake(IntakeConstants.INTAKE_SPEED));
		xbox.povRight().whileTrue(intake.extendOut());
		xbox.povLeft().whileTrue(intake.extendIn());

		xbox.leftBumper().whileTrue(shooter.zeroHood()).onFalse(shooter.resetEncoder());
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
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
	
	public void updates() {
		vision.updateDashboard();
	}
}
