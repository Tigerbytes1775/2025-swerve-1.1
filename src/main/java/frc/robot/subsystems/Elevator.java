package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;



public class Elevator extends SubsystemBase {
    
    private final SparkMax elevatorMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);

    private final double elevatorSpeed = 0.0001;

    public final PidController pidController = new PidController(0, 0.1, 0, elevatorMotor);

    


    
    public Elevator() {}

    public void setMotors(double percent) {
        
        elevatorMotor.set(percent * elevatorSpeed);

        SmartDashboard.putNumber("Elevator Power(%)", percent * elevatorSpeed);

        if(percent == 0) {
            elevatorMotor.stopMotor();
        }
    }

    public void setTarget(double target) {
        pidController.setTargetPoint(target);
    }

    
    

    public void update() {
        pidController.update();
        setMotors(pidController.GetForce());
    }

    public static Command GetElevatorCommand(Elevator elevator, double target) {
        return new Command() {
            

            @Override
            public void initialize() {

                elevator.pidController.setTargetPoint(target);
            }

            @Override
            public void execute() {
                elevator.update();
            }

            @Override
            public boolean isFinished() {
                return elevator.pidController.IsAtTarget();
            }
        };
    }
}
