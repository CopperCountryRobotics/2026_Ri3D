package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.CLIMBER_ID;

public class ClimberSubsystem extends SubsystemBase {
   private final SparkMax motor = new SparkMax(CLIMBER_ID, MotorType.kBrushless);
   private double setpoint = 0;

   private final PIDController controller = new PIDController(0, 0, 0);
   private final ElevatorFeedforward ffeController = new ElevatorFeedforward(6, 0.1, 0);

    /**Constructor */
    public ClimberSubsystem(){}

    /**method to change the setpoint of the elevator */
   public Command setClimber(double height){
      return runOnce(()->{
         setpoint = height;
      });
   }

   /**method to calculate effort with controllers */
   private double calculateEffort(){
      return ffeController.calculate(0) + controller.calculate(motor.getEncoder().getPosition(), setpoint);
   }

    @Override
    public void periodic(){
      //motor.setVoltage(calculateEffort());//TODO add when testing irl
      SmartDashboard.putNumber("Climber height", motor.getEncoder().getPosition());
      SmartDashboard.putNumber("Climber setpoint", setpoint);
    }
}
