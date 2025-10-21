package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeIntake extends SubsystemBase {

    
    private final SparkMax algaeMotor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);

    private final double algeaStrength = 0.45;
    
    
    public AlgaeIntake() {}

    public void setMotors(double percent) {
        double power = percent * algeaStrength;

        SmartDashboard.putNumber("Algae intake power(%)", power);

        if (percent == 0){
            algaeMotor.stopMotor();
        } else {
            algaeMotor.set(power);
        }
        
    }
    
    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {
            setMotors(
              controller.getLeftBumperButton() ? 
              -1 : controller.getRightBumperButton() ? 
              1 : 0
            );
          }
        );
    }

    public static Command GetAlgaeIntakeCommand(AlgaeIntake algaeIntake, double percent, double commandTime) {
        return new Command() {
        
            private final Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                algaeIntake.setMotors(percent);
            }

            @Override
            public boolean isFinished() {
                return timer.get() <= commandTime;
            }
        };

    }
}
