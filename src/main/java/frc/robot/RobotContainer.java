package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveCmd;
import frc.robot.joysticks.Driver;
import frc.robot.subsystems.SwerveSubsystem;
//import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

	//private final Pigeon2 gyro = new Pigeon2(9);

	private final Driver driver = new Driver();
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	//private final VisionSubsystem visionSubsystem = new VisionSubsystem(
		//this.swerveSubsystem::addVisionMeasurement, this.swerveSubsystem::getPose);

		private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(
			new SwerveCmd(
				this.swerveSubsystem,
				this.driver::getXDesiredSpeed, this.driver::getYDesiredSpeed, this.driver::getRDesiredSpeed));

		//register named commands here

		//config pathplanner
		swerveSubsystem.configPathPlanner();
		//add auto chooser
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void configBindings() {
		this.driver.resetGyro()
			.onTrue(Commands.runOnce(this.swerveSubsystem::resetGyro, this.swerveSubsystem));
		this.driver.resetPosition()
			.onTrue(Commands.runOnce(this.swerveSubsystem::resetSwerveEncoders, this.swerveSubsystem));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
