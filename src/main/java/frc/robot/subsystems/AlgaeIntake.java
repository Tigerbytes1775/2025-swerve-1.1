package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;


public class AlgaeIntake extends SubsystemBase {

    
    private final SparkMax algaeMotor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);

    double algeaStrength = 1;
    
    
    public AlgaeIntake() {}

    public void setMotors(double percent) {
        double power = percent * algeaStrength;
        //algaeMotor.set(power);

        SmartDashboard.putNumber("Algae intake power(%)", power);

        if (percent == 0){
            //algaeMotor.stopMotor();

        }
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
