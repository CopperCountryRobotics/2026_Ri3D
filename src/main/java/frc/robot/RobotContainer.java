package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.SwerveConstants.DEAD_BAND;

public class RobotContainer {
	// Joysticks
	private final CommandXboxController xbox = new CommandXboxController(0);

	// Subsystems/custom class instiantiation
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		swerveSubsystem.setDefaultCommand(
				Commands.run(() -> swerveSubsystem.drive(
						(-MathUtil.applyDeadband(xbox.getLeftY(), DEAD_BAND) * 5
								* MathUtil.applyDeadband(xbox.getRightTriggerAxis(), DEAD_BAND)),
						(-MathUtil.applyDeadband(xbox.getLeftX(), DEAD_BAND) * 5
								* MathUtil.applyDeadband(xbox.getRightTriggerAxis(), DEAD_BAND)),
						(-MathUtil.applyDeadband(xbox.getRightX(), DEAD_BAND) * 5
								* MathUtil.applyDeadband(xbox.getRightTriggerAxis(), DEAD_BAND)),
						true)));

		configBindings();

		// register named commands here

		// config pathplanner
		swerveSubsystem.configPathPlanner();
		// add auto chooser to dashboard
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		xbox.a().onTrue(swerveSubsystem.resetGyro());
		xbox.b().onTrue(swerveSubsystem.resetPose(new Pose2d(0, 0, new Rotation2d(0))));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
