package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;



public class Elevator extends SubsystemBase {
    
    private final SparkFlex elevatorMotor;

    private final double elevatorSpeed = 0.3;

    //private final PidController pidController;
    private final PIDController pidController;

    public final double L1Height = 0;
    public final double L2Height = 25;
    public final double L3Height = 50;
    public final double L4Height = 80;



    
    public Elevator() {
        elevatorMotor  = new SparkFlex(32, SparkLowLevel.MotorType.kBrushless);
        //pidController = new PidController(0.3, 0.1, 0, elevatorMotor);
        pidController = new PIDController(0.3, 0.1, 0);
    }

    public void setMotors(double percent) {
        
        elevatorMotor.set(-percent * elevatorSpeed);
        

        SmartDashboard.putNumber("Elevator Power(%)", percent * elevatorSpeed);
        SmartDashboard.putNumber("Elevator Pos", elevatorMotor.getEncoder().getPosition());

        if(percent == 0) {
            elevatorMotor.stopMotor();
        }
    }

    public void setTarget(double target) {
        //pidController.setTargetPoint(target);
        pidController.setSetpoint(target);
        
    }

    
    

    public void update() {
        
        setMotors(pidController.calculate(elevatorMotor.getEncoder().getPosition()));

    }

    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {

            boolean aButton = controller.getAButtonPressed();
            boolean bButton = controller.getBButtonPressed();
            boolean xButton = controller.getXButtonPressed();
            boolean yButton = controller.getYButtonPressed();

            boolean anyButton = aButton || bButton || xButton || yButton;

            if(anyButton) {
                setTarget(
                    aButton ? 
                    L1Height : bButton ?
                    L2Height : xButton ?
                    L3Height : L4Height
                );
            }
            
            update();
        });
    }


    public static Command GetAutoCommand(Elevator elevator, double target) {
        return new Command() {
            

            @Override
            public void initialize() {

                elevator.setTarget(target);
            }

            @Override
            public void execute() {
                elevator.update();
            }

            @Override
            public boolean isFinished() {
                return elevator.pidController.atSetpoint();
            }
        };
    }
}
