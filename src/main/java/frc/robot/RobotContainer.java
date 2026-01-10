package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.XboxButtonValues.*;

public class RobotContainer {
	// Joysticks
	private final XboxController xbox = new XboxController(0);

	// Subsystems/custom class instiantiation
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(xbox, true);

	// Sendable chooser for auton (appears on Dashboards)
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		configBindings();

		// register named commands here

		// config pathplanner
		swerveSubsystem.configPathPlanner();
		// add auto chooser to dashboard
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		new JoystickButton(xbox, A).onTrue(swerveSubsystem.resetGyro());
		new JoystickButton(xbox, B).onTrue(swerveSubsystem.resetPose(new Pose2d(0, 0, new Rotation2d(0))));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
