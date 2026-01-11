package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.subsystems.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.DEAD_BAND;
import static frc.robot.Constants.SwerveConstants.KINEMATICS;
import static frc.robot.Constants.SwerveConstants.MAX_SPEED;
import static frc.robot.Constants.HarwareConstants.*;


import java.util.function.Supplier;

/**
 * Swerve Subsystem class that creates a swerve drivetrain using Swerve
 * Modules(NEO motors) and a ctre pigeon2
 */
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ENCODER_PORT,
            false, true,
            "FrontLeft");
    private final SwerveModule frontRight = new SwerveModule(
            FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ENCODER_PORT,
            true, true,
            "FrontRight");
    private final SwerveModule backLeft = new SwerveModule(
            BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, BACK_LEFT_ENCODER_PORT,
            false, true,
            "BackLeft");
    private final SwerveModule backRight = new SwerveModule(
            BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, BACK_RIGHT_ENCODER_PORT,
            true, true,
            "BackRight");
    private final Pigeon2 gyro = new Pigeon2(GYRO_ID);

    private final XboxController xbox;
    private final boolean fieldOriented;

    private final StructPublisher<Pose2d> swervePose = NetworkTableInstance.getDefault()
            .getStructTopic("AdvantageScope/SwervePose", Pose2d.struct).publish();

    Vector<N3> stateStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
    Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            KINEMATICS,
            this.getRotation2d(),
            this.getSwervePosition(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);

    // private final Supplier<Double> xSpeed, ySpeed, rSpeed;

    private final SendableChooser<Double> polarityChooserX = new SendableChooser<>();
    private final SendableChooser<Double> polarityChooserY = new SendableChooser<>();

    /** Standard constructor */
    public SwerveSubsystem(XboxController xbox, boolean fieldOriented) {
        super("Swerve", false);

        // this.xSpeed = ()-> xSpeed.get() * 2;
        // this.ySpeed = ()->ySpeed.get()*2;
        // this.rSpeed = ()->rSpeed.get()*2;

        polarityChooserX.setDefaultOption("Positive", 1.0);
        polarityChooserX.addOption("Negative", -1.0);
        SmartDashboard.putData("Polarity Chooser X", polarityChooserX);

        polarityChooserY.setDefaultOption("Positive", 1.0);
        polarityChooserY.addOption("Negative", -1.0);
        SmartDashboard.putData("Polarity Chooser Y", polarityChooserY);

        this.xbox = xbox;
        this.fieldOriented = fieldOriented;
    }

    /** drive method, built for use with a controller */
    public Command drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rSpeed,
            Supplier<Boolean> fieldOriented) {
        return run(() -> {
            SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
                    fieldOriented.get().booleanValue()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed.get().doubleValue() * polarityChooserX.getSelected(),
                                    ySpeed.get().doubleValue() * polarityChooserY.getSelected(),
                                    rSpeed.get().doubleValue(), this.getRotation2d())
                            : new ChassisSpeeds(xSpeed.get().doubleValue(), ySpeed.get().doubleValue(),
                                    rSpeed.get().doubleValue()));
            this.setDesiredStates(states);
        });
    }

    /** drive method, built for auto use */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(speeds);
        this.setDesiredStates(states);
    }

    public double getHeading() {
        return this.gyro.getRotation2d().getDegrees();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(this.getHeading()));
    }

    public ChassisSpeeds getSpeeds() {
        return KINEMATICS.toChassisSpeeds(this.getSwerveState());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public Command resetPose(Pose2d pose) {
        return runOnce(() -> {
            this.poseEstimator.resetPose(pose);
        });
    }

    public Command resetGyro() {
        return runOnce(() -> {
            gyro.reset();
        });
    }

    public void resetSwerveEncoders() {
        this.frontLeft.resetEncoder();
        this.frontRight.resetEncoder();
        this.backLeft.resetEncoder();
        this.backRight.resetEncoder();
    }

    public SwerveModuleState[] getSwerveState() {
        return new SwerveModuleState[] {
                this.frontLeft.getState(),
                this.frontRight.getState(),
                this.backLeft.getState(),
                this.backRight.getState()
        };
    }

    public SwerveModulePosition[] getSwervePosition() {
        return new SwerveModulePosition[] {
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        System.out.println("A");
        this.poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        this.poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    /** Pathplanner configuration - must be called once in Robot Container */
    public void configPathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> getPose(),
                    this::resetPose,
                    () -> getSpeeds(),
                    (speeds, feedforwards) -> drive(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(10, 0, 0), // drive
                            new PIDConstants(7, 0, 0)), // Rotation
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError("Pathplanner failed to configure", ex.getStackTrace());
        }
    }

    // updates dashboard
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("GyroAngle", this.getHeading());
        SmartDashboard.putString("PoseEstimator", this.poseEstimator.getEstimatedPosition().toString());
    }

    // updates pose and driving
    @Override

    public void periodic() {
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
                fieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                MathUtil.applyDeadband(xbox.getLeftY(), DEAD_BAND) * 5 * polarityChooserX.getSelected(),
                                MathUtil.applyDeadband(xbox.getLeftX(), DEAD_BAND) * 5
                                        * polarityChooserY.getSelected(),
                                MathUtil.applyDeadband(xbox.getRightX(), DEAD_BAND) * 5, this.getRotation2d())
                        : new ChassisSpeeds(
                                MathUtil.applyDeadband(xbox.getLeftY(), DEAD_BAND) * 5 * polarityChooserX.getSelected(),
                                MathUtil.applyDeadband(xbox.getLeftX(), DEAD_BAND) * 5
                                        * polarityChooserY.getSelected(),
                                MathUtil.applyDeadband(xbox.getRightX(), DEAD_BAND) * 5));
        this.setDesiredStates(states);

        this.poseEstimator.update(
                this.getRotation2d(), this.getSwervePosition());
        this.swervePose.accept(this.poseEstimator.getEstimatedPosition());
    }
}
