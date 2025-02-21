package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;



public class Elevator extends SubsystemBase {
    
    private final SparkMax elevatorMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);

    private final double elevatorSpeed = 0.0001;

    public final  PidController pidController = new PidController(0, 0.1, 0, elevatorMotor);

    
    public Elevator() {}

    public void setMotors(double percent) {
        
        elevatorMotor.set(percent * elevatorSpeed);

        SmartDashboard.putNumber("Elevator Power(%)", percent * elevatorSpeed);

        if(percent == 0) {
            elevatorMotor.stopMotor();
        }
    }
    
}
