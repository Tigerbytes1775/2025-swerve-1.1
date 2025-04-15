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
    private final DoubleSupplier posSupplier;
    private final RelativeEncoder encoder;

    private final double manualSpeed = 0.6;
    private final double pidSpeedUp = 0.4;
    private final double pidSpeedDown = 0.15;

    //private final PidController pidController;
    public final PIDController pidController;

    public final double L1Height = 0;
    public final double L2Height = 22;
    public final double L3Height = 49.5;
    public final double L4Height = 96;

    public boolean pidEnabled = false;
    
    public Elevator() {
        elevatorMotor  = new SparkFlex(32, SparkLowLevel.MotorType.kBrushless);
        encoder = elevatorMotor.getEncoder();
        posSupplier = encoder::getPosition;
        zeroEncoder();

       
        pidController = new PIDController(
            0.1,
            0.01,
            0.01
        );

        
    }

    public void setMotors(double percent) {
        double power = -percent;
        elevatorMotor.set(power);

        SmartDashboard.putNumber("Elevator Power(%)", power);
        SmartDashboard.putNumber("Elevator Pos", elevatorMotor.getEncoder().getPosition());
        //SmartDashboard.putBoolean("Set Target", false);

        if(percent == 0) {
            elevatorMotor.stopMotor();
        }
    }

    public void setTarget(double target) {
        //SmartDashboard.putBoolean("Set Target", true);
        pidController.setSetpoint(target);

        if(posSupplier.getAsDouble() < 3) {
            zeroEncoder();
        }
        
        
        
    }

    public void zeroEncoder() {
        elevatorMotor.getEncoder().setPosition(L1Height);
    }
    

    public void update() {
        double power = -pidController.calculate(posSupplier.getAsDouble());
        power *= power <= 0 ? pidSpeedUp : pidSpeedDown;
        SmartDashboard.putNumber("Pid Power", -power);
        setMotors(power);

    }

    public Command GetTeleopCommand(XboxController controller) {
        Command command = run(() -> {

            double rightY = MathUtil.applyDeadband(controller.getRightY(), 0.1);

            boolean aButton = controller.getAButtonPressed();
            boolean bButton = controller.getBButtonPressed();
            boolean xButton = controller.getXButtonPressed();
            boolean yButton = controller.getYButtonPressed();

            boolean anyPidButton =  aButton || bButton || xButton || yButton;


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
                setMotors(rightY * manualSpeed);
            }
            
            
        });

        
        //command.beforeStarting(runOnce(() -> zeroEncoder()));

        return command;
    }


    public Command GetAutoCommand(Elevator elevator, double target) {
        return new Command() {
            

            @Override
            public void initialize() {
                
                elevator.setTarget(target);
            }

            @Override
            public void execute() {

                //System.out.println("Elevator Auto On");
                elevator.update();
            }

            @Override
            public boolean isFinished() {
                return elevator.pidController.atSetpoint();
            }
        };
    }
}
