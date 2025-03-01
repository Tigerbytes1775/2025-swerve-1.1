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

        if(leftY >= 0.5) {
            algaePivot.setTarget(1);
        } else if(leftY <= -0.5) {
            algaePivot.setTarget(0);
        }

        algaePivot.update();
    }
}
