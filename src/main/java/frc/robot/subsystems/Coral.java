package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
}
