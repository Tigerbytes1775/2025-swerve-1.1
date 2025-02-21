package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.PidController;

public class TeleopAlgaePivotCommand extends Command {
    
    private final AlgaePivot algaePivot;
    private final DoubleSupplier leftY;
    private final PidController pidController;

    public TeleopAlgaePivotCommand(AlgaePivot algaePivot, DoubleSupplier leftY) {
        addRequirements(algaePivot);
        this.algaePivot = algaePivot;
        this.leftY = leftY;
        this.pidController = algaePivot.pidController;
    }

    @Override
    public void execute() {

        double leftY = this.leftY.getAsDouble();

        if(leftY >= 0.5) {
            pidController.setTargetPoint(10);
        } else if(leftY <= -0.5) {
            pidController.setTargetPoint(0);
        }

        pidController.update();
        algaePivot.setMotors(pidController.GetForce());
    }
}
