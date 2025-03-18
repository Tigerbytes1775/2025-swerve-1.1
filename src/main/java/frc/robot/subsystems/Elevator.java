package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;





public class Elevator extends SubsystemBase {
    
    private final SparkFlex elevatorMotor;
    private final DoubleSupplier pos;
    private final RelativeEncoder encoder;

    private final double elevatorSpeed = 0.5;

    //private final PidController pidController;
    private final PIDController pidController;

    public final double L1Height = 0;
    public final double L2Height = 18;
    public final double L3Height = 34;
    public final double L4Height = 69;

    public boolean pidEnabled = false;
    
    public Elevator() {
        elevatorMotor  = new SparkFlex(32, SparkLowLevel.MotorType.kBrushless);
        encoder = elevatorMotor.getEncoder();
        pos = encoder::getPosition;
        elevatorMotor.getEncoder().setPosition(L1Height);

        //pidController = new PidController(0.3, 0.1, 0, elevatorMotor);
        pidController = new PIDController(
            SmartDashboard.getNumber("Elevator PID p", 0.005),
            SmartDashboard.getNumber("Elevator PID i", 0.01),
            SmartDashboard.getNumber("Elevator PID d", 0.0)
        );

        
    }

    public void setMotors(double percent) {
        
        elevatorMotor.set(-MathUtil.applyDeadband(percent, 0.02) * elevatorSpeed);

        SmartDashboard.putNumber("Elevator Power(%)", percent * elevatorSpeed);
        SmartDashboard.putNumber("Elevator Pos", elevatorMotor.getEncoder().getPosition());

        if(percent == 0) {
            elevatorMotor.stopMotor();
        }
    }

    public void setTarget(double target) {
        //pidController.setTargetPoint(target);

        double pos = this.pos.getAsDouble();
        pidController.setSetpoint(target);
        if(pos < L1Height + 1) {
            encoder.setPosition(L1Height);
        }
        
        
    }

    public void zeroEncoder() {
        elevatorMotor.getEncoder().setPosition(L1Height);
    }
    

    public void update() {
        
        setMotors(pidController.calculate(elevatorMotor.getEncoder().getPosition()));

    }

    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {

            double rightY = MathUtil.applyDeadband(controller.getRightY(), 0.1);

            boolean aButton = controller.getAButtonPressed();
            boolean bButton = controller.getBButtonPressed();
            boolean xButton = controller.getXButtonPressed();
            boolean yButton = controller.getYButtonPressed();

            boolean anyPidButton = aButton || bButton || xButton || yButton;


            pidEnabled = 
                rightY != 0 ?
                false : anyPidButton ?
                true : pidEnabled;
                


            if(pidEnabled) {
                
                if(anyPidButton) {
                    setTarget(
                        aButton ? 
                        L1Height : bButton ?
                        L2Height : xButton ?
                        L3Height : L4Height
                    );
                }
                
                update();
            } else {
                setMotors(rightY);
            }
            
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
