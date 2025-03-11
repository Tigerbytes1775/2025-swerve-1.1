package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaePivot extends SubsystemBase {
    
    private final SparkFlex algaeMotor;

    private final double algeaStrengthUp = 0.1;
    private final double algeaStrengthDown = 0.15;

    private final double downTarget = 10;
    private final double upTarget = 30;

    private final PIDController pidController;


    public AlgaePivot() {
        algaeMotor = new SparkFlex(30, SparkLowLevel.MotorType.kBrushless);

        //algaeMotor.configure(SparkBaseConfig.IdleMode.kBrake, SparkBase.ResetMode., null);
        pidController = new PIDController(0,0, 0);
        
    }

    public void setMotors(double percent) {
        double power = 0;
        if(percent < 0 )  {
            power = algeaStrengthDown;
        } else if (percent > 0) {
            power = -algeaStrengthUp;
        } 

        SmartDashboard.putNumber("Algae Pivot Power(%)", power);
        SmartDashboard.putNumber("Algae Pivot Pos", algaeMotor.getAbsoluteEncoder().getPosition());

        if(percent == 0){
            algaeMotor.stopMotor();
        } else {
            algaeMotor.set(power);
        }
    }

    public void setTarget(double target) {
        pidController.setSetpoint(target);
    }

    public void update() {
        setMotors(pidController.calculate(algaeMotor.getAbsoluteEncoder().getPosition()));
    }

    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {
            double leftY = controller.getLeftY();

            boolean isUp = leftY > 0.5;
            boolean isDown = leftY < -0.5;

            if(isUp || isDown) {
                setTarget(isDown ? downTarget : upTarget);
            }
            
            update();
        }
      );
    }

    public static Command GetAlgaePivotCommand(AlgaePivot algaePivot, double target) {
        return new Command() {
            

            @Override
            public void initialize() {
                algaePivot.setTarget(target);
            }

            @Override
            public void execute() {
                algaePivot.update();
            }

            @Override
            public boolean isFinished() {
                return algaePivot.pidController.atSetpoint();
            }
        };
    }
}
