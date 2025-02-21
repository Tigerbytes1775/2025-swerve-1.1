package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;

public class AlgaePivot extends SubsystemBase {
    
    private final SparkMax algaeMotor = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);

    private final double algeaStrength = 1;

    public final PidController pidController = new PidController(0,0.1, 0, algaeMotor);


    public AlgaePivot() {}

    public void setMotors(double percent) {
        double power = percent * algeaStrength;
        algaeMotor.set(power);

        SmartDashboard.putNumber("Algae Pivot Power(%)", power);

        if(percent == 0){
            algaeMotor.stopMotor();
        }
    }
}
