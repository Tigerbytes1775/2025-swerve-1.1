package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    
    private final double coralPower = 0.8;

    private final PWMVictorSPX coralMoter1 = new PWMVictorSPX(7);
    private final PWMVictorSPX coralMoter2 = new PWMVictorSPX(8);

    public Coral() {

        coralMoter1.setInverted(true);
        coralMoter2.setInverted(false);

    }

    public void setMotors(double percent) {
        coralMoter1.set(percent * coralPower);
        coralMoter2.set(percent * coralPower);

        SmartDashboard.putNumber("Coral Power(%)", percent * coralPower);
        if(percent == 0) {
            coralMoter1.stopMotor();
            coralMoter2.stopMotor();
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

