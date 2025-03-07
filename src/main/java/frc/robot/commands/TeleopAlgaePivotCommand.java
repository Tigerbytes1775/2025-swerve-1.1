package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

public class TeleopAlgaePivotCommand extends Command {
    
    private final AlgaePivot algaePivot;
    private final DoubleSupplier leftY;

    public TeleopAlgaePivotCommand(AlgaePivot algaePivot, DoubleSupplier leftY) {
        addRequirements(algaePivot);
        this.algaePivot = algaePivot;
        this.leftY = leftY;
    }

    @Override
    public void execute() {

        double leftY = this.leftY.getAsDouble();
        double power = 0;
        if(leftY >= 0.5) {
            //algaePivot.setTarget(0);
            power = -leftY;
        } else if(leftY <= -0.5) {
            power = -leftY;
            //algaePivot.setTarget(1);
        }
        algaePivot.setMotors(power);

        //algaePivot.update();
    }
}
