package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;



public class Elevator extends SubsystemBase {
    
    private final SparkMax elevatorMotor;

    private final SparkSim elevatorMotorSim;

    private final double elevatorSpeed = 1;

    public final PidController pidController;

    private final DCMotor elevatorMotorDC = new DCMotor(elevatorSpeed, elevatorSpeed, elevatorSpeed, elevatorSpeed, elevatorSpeed, 1);


    
    public Elevator() {
        elevatorMotor  = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
        
        elevatorMotorSim = new SparkSim(elevatorMotor, elevatorMotorDC);

        pidController = new PidController(0.02, 0, 0, elevatorMotorSim);
        elevatorMotorSim.enable();
    }

    public void setMotors(double percent) {
        
        elevatorMotorSim.setAppliedOutput(percent * elevatorSpeed);
        
        SmartDashboard.putNumber("Elevator Motor Pos", elevatorMotorSim.getPosition());
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
