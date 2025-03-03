package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    
    private final double coralPowerOut = 1;
    private final double coralPowerIn = 0.05;

    private final SparkFlex coralMotorIn = new SparkFlex(28, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax coralMotorOut = new SparkMax(29, SparkLowLevel.MotorType.kBrushless);

    public Coral() {}

    public void setMotors(double percent) {

        

        

        SmartDashboard.putNumber("Coral Power(%)", percent);
        if(percent < 0) {
            coralMotorIn.set(-percent * coralPowerIn);
        } else {
            coralMotorOut.set(-percent * coralPowerOut);
        }
    }

    public static Command GetCoralCommand(Coral coral, double percent, double commandTime) {
        return new Command() {
            
            private final Timer timer = new Timer();
            
            @Override
            public void initialize() {
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                coral.setMotors(percent);
            }

            @Override
            public boolean isFinished() {
                return timer.get() <= commandTime;
            }
        };

    }
}   

