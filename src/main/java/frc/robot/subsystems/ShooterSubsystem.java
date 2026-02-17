package frc.robot.subsystems;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.MotorType;
import com.thethriftybot.devices.ThriftyNova.PIDSlot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.HardwareConstants.GATE_MOTOR_ID;
import static frc.robot.Constants.HardwareConstants.HOOD_MOTOR_ID;
import static frc.robot.Constants.HardwareConstants.HOOD_SWITCH;
import static frc.robot.Constants.HardwareConstants.SHOOTER_ID;

public class ShooterSubsystem extends SubsystemBase {
    private final ThriftyNova shooterMotor;
    private final ThriftyNova hoodMotor;
    private final ThriftyNova gateMotor;
    private final DigitalInput hoodSwitch;

    private Vision vision;
    private CommandXboxController xbox;
    private double hoodSetpoint;
    private double setPos;//for lerp filling testing
    private final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();

    private double liveHoodOffset = 0;

    private double setSpeed = 0;

    private final SendableChooser<Boolean> autoAdjustChooser;

    /** Constructor */
    public ShooterSubsystem(Vision vision, CommandXboxController xbox) {

        shooterMotor = new ThriftyNova(SHOOTER_ID, MotorType.NEO);
        shooterMotor.useEncoderType(EncoderType.INTERNAL);
        shooterMotor.pid0.setP(0);
        shooterMotor.pid0.setI(0.000);
        shooterMotor.pid0.setD(0.0);
        shooterMotor.pid0.setFF(0.007); // Is probably a little under the nominal amount
        shooterMotor.pid0.setAccumulatorCap(0.05);
        shooterMotor.usePIDSlot(PIDSlot.SLOT0);
        shooterMotor.setMaxCurrent(CurrentType.SUPPLY, 40);
        shooterMotor.setMaxCurrent(CurrentType.STATOR, 40);


        setPos = shooterMotor.getSetPoint();

        hoodMotor = new ThriftyNova(HOOD_MOTOR_ID, MotorType.NEO);

        // hoodMotor.setRampUp(0.0125);
        // hoodMotor.setRampDown(0.02);

        hoodMotor.useEncoderType(EncoderType.INTERNAL);
        hoodMotor.setEncoderPosition(0);
        hoodMotor.pid1.setP(0.3);// TODO add back
        hoodMotor.pid1.setI(0.0000);
        hoodMotor.pid1.setD(0.03);
        hoodMotor.pid1.setFF(0.007);// TODO addback
        hoodMotor.pid1.setAccumulatorCap(0.00005);
        hoodMotor.pid1.setAllowableError(0.00001);

        hoodMotor.usePIDSlot(PIDSlot.SLOT1);

        hoodSwitch = new DigitalInput(HOOD_SWITCH);

        gateMotor = new ThriftyNova(GATE_MOTOR_ID, MotorType.NEO);

        autoAdjustChooser = new SendableChooser<>();
        autoAdjustChooser.setDefaultOption("auto adjust", true);
        autoAdjustChooser.addOption("driver operated", false);
        SmartDashboard.putData(autoAdjustChooser);



        // Lerp table config
        lerpTable.put(0.0, 0.0);// TODO add more

        this.xbox = xbox;
    }

    /** Command to "set and forget" the shooter motor speed, in rev/sec */
    public Command setShooter(double speed) {
        return runOnce(() -> {
            shooterMotor.setVelocity(speed);
            setSpeed = speed;
        });
    }

    public Command holdShooter(double speed) {
        return run(() -> {
            shooterMotor.setVelocity(speed);
            setSpeed = speed;
        });
    }

    /** Command with end statement to set the motor speed to zero */
    public Command runShooter(double speed) {
        return runEnd(() -> {
            shooterMotor.set(speed);
        }, () -> {
            shooterMotor.set(0);
        });
    }


    /** returns the shooter motors speed in revolutions per second */
    public double getShooterSpeed() {
        return shooterMotor.getVelocity();
    }

    /** run once command to set the speed of the gate motor */
    public Command setGate(double speed) {
        return runOnce(() -> {
            gateMotor.set(speed);
        });
    }

    /** returns the position of the hood in degrees */
    public double getHoodPosition() {
        return hoodMotor.getPosition() * 360;
    }

    public Command zeroHood(){
        return runEnd(()->{
            hoodMotor.setVoltage(-0.7);
        }, ()->{
            hoodMotor.set(0);
            //hoodMotor.setEncoderPosition(0);
        });
    }

    public Command resetEncoder(){
        return runOnce(()->{
            Commands.sequence(
                Commands.waitSeconds(3)
            );
            hoodMotor.setEncoderPosition(0);
        });
    }

    /** Updates the position of the hood */
    public Command setHood(double position) {
        return runOnce(() -> {
            setPos = position;
            hoodMotor.setPosition(position - liveHoodOffset);
        });
    }

    public Command bumpHoodUp(){
        return runOnce(() -> {
            hoodMotor.setPosition(setPos+0.1);
            setPos = setPos+0.1;
        });
    }

    public Command bumpHoodDown(){
        return runOnce(() -> {
            hoodMotor.setPosition(setPos-0.1);
            setPos=setPos-0.1;
        });
    }

    /**
     * run end command to move the hood motor at a percent speed, and to zero upon
     * ending
     */
    public Command runHood(double speed) {
        return runEnd(() -> {
            hoodMotor.set(speed);
        }, () -> {
            hoodMotor.set(0);
        });
    }

    public Command setHoodSpeed(double speed){
        return runOnce(()->{
            hoodMotor.set(speed);
        });
    }

    @Override
    public void periodic() {
        //TODO implement after filling lerp
        // if(autoAdjustChooser.getSelected()){
        //     if(vision.getSkew()!=0){
        //         hoodSetpoint = lerpTable.get(vision.getSkew());
        //     }
        //     hoodMotor.setPosition(hoodSetpoint);
        // } else {
        //     if(xbox.rightBumper().getAsBoolean()){
        //         setHoodSpeed(0.4);
        //     } else if (xbox.leftBumper().getAsBoolean()){
        //         setHoodSpeed(-0.4);
        //     } else {
        //         setHoodSpeed(0);
        //     }
        // }
        // update dashboard
        SmartDashboard.putNumber("Shooter speed", this.shooterMotor.getVelocity());
        SmartDashboard.putNumber("Shooter set speed", setSpeed);
        SmartDashboard.putNumber("Hood encoder", hoodMotor.getPositionInternal());
        SmartDashboard.putNumber("Hood encoder rotations?", hoodMotor.getPositionInternal() / (4096 / 25));
        SmartDashboard.putNumber("Hood Goal Pos", setPos);//for lerp filling testing

    }
}
