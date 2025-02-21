package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeIntake extends SubsystemBase {

    
    private final SparkMax algaeMotor = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);

    double algeaStrength = 1;
    
    
    public AlgaeIntake() {}

    public void setMotors(double percent) {
        double power = percent * algeaStrength;
        algaeMotor.set(power);

        SmartDashboard.putNumber("Algae intake power(%)", power);

        if (percent == 0){
            algaeMotor.stopMotor();

        }
    }
}
