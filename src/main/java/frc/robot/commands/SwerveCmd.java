package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCmd extends Command {
	private final SwerveSubsystem swerveSubsystem;
	private final Supplier<Double> xSpeed, ySpeed, rSpeed;

	private final SendableChooser<Double> polarityChooserX = new SendableChooser<>();
	private final SendableChooser<Double> polarityChooserY = new SendableChooser<>();


	public SwerveCmd(
		SwerveSubsystem swerveSubsystem,
		Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rSpeed) {
		this.swerveSubsystem = swerveSubsystem;
		this.xSpeed = ()-> xSpeed.get() * 2;
		this.ySpeed = ()->ySpeed.get()*2;
		this.rSpeed = ()->rSpeed.get()*2;
		this.addRequirements(this.swerveSubsystem);

		polarityChooserX.setDefaultOption("Positive", 1.0);
		polarityChooserX.addOption("Negative", -1.0);
		SmartDashboard.putData("Polarity Chooser X", polarityChooserX);

		polarityChooserY.setDefaultOption("Positive", 1.0);
		polarityChooserY.addOption("Negative", -1.0);
		SmartDashboard.putData("Polarity Chooser Y", polarityChooserY);

	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.swerveSubsystem.drive(this.xSpeed.get()*polarityChooserX.getSelected(), this.ySpeed.get()*polarityChooserY.getSelected(), this.rSpeed.get(), true);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
