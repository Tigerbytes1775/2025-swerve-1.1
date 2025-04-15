package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    
    private final double coralPowerOut = 0.5;
    private final double coralPowerIn = 0.1;

    private final SparkFlex coralMotorIn = new SparkFlex(28, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax coralMotorOut = new SparkMax(29, SparkLowLevel.MotorType.kBrushless);

    public Coral() {}

    public void setInMotor(double percent){
        double power = percent * coralPowerIn;
        coralMotorIn.set(power);
        SmartDashboard.putNumber("Coral Intake Motor(%)", power);
    }

    public void setOutMotor(double percent){
        double power = -percent * coralPowerOut;
        
        SmartDashboard.putNumber("Coral OutMotor(%)", power);
        coralMotorOut.set(power);

    }

    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {

            setInMotor(
              controller.getRightTriggerAxis() > 0.1 ? 1 : 0
            );
            
            setOutMotor(
              controller.getLeftTriggerAxis() > 0.1 ? 
              1 : controller.getPOV() != -1 && controller.getPOV() != 180?
              -0.2 : 0
            );
    
          }
        );
    }

    public static Command GetCoralOutCommand(Coral coral, double percent, double commandTime) {
        return new Command() {
            
            private final Timer timer = new Timer();
            
            @Override
            public void initialize() {
                coral.setOutMotor(percent);
                timer.reset();
                timer.start();
            }

            @Override
            public void execute() {
                coral.setOutMotor(percent);
            }

            @Override
            public boolean isFinished() {
                return timer.get() >= commandTime;
            }

            @Override
            public void end(boolean interrupted) {
                coral.setOutMotor(0);
            }

            
        };

    }

    public static Command GetCoralInCommand(Coral coral, double percent, double commandTime) {
        return new Command() {
            
            private final Timer timer = new Timer();
            
            @Override
            public void initialize() {
                coral.setInMotor(percent);
                timer.reset();
                timer.start();
            }

            //@Override
            //public void execute() {
            //    coral.setInMotor(percent);
            //}

            @Override
            public boolean isFinished() {
                return timer.get() >= commandTime;
            }

            @Override
            public void end(boolean interrupted) {
                coral.setInMotor(0);
            }
        };

    }
}   

