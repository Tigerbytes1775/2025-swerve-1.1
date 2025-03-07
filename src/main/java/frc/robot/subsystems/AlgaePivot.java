package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;

public class AlgaePivot extends SubsystemBase {
    
    private final SparkFlex algaeMotor;

    private final double algeaStrengthUp = 0.15;
    private final double algeaStrengthDown = 0.1;

    private final PidController pidController;


    public AlgaePivot() {
        algaeMotor = new SparkFlex(30, SparkLowLevel.MotorType.kBrushless);
        pidController = new PidController(SmartDashboard.getNumber("Pivot p:", 0.1),
        SmartDashboard.getNumber("Pivot i:", 0.1), 0, algaeMotor);
    }

    public void setMotors(double percent) {
        double power = 0;
        if(percent < 0 )  {
            power = -algeaStrengthDown;
        } else if (percent > 0) {
            power = algeaStrengthUp;
        }

        algaeMotor.set(power);

        SmartDashboard.putNumber("Algae Pivot Power(%)", power);
        SmartDashboard.putNumber("Algae Pivot Pos", algaeMotor.getAbsoluteEncoder().getPosition());

        if(percent == 0){
            algaeMotor.stopMotor();
        }
    }

    public void setTarget(double target) {
        pidController.setTargetPoint(target);
    }

    public void update() {
        pidController.update();
        setMotors(pidController.GetForce());
    }

    public static Command GetAlgaePivotCommand(AlgaePivot algaePivot, double target) {
        return new Command() {
            

            @Override
            public void initialize() {
                algaePivot.pidController.setTargetPoint(target);
            }

            @Override
            public void execute() {
                algaePivot.update();
            }

            @Override
            public boolean isFinished() {
                return algaePivot.pidController.IsAtTarget();
            }
        };
    }
}
