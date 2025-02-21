package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PidController;
import frc.robot.subsystems.Elevator;

public class TeleopElevatorCommand extends Command{
    
    private final Elevator elevator;
    private final BooleanSupplier bButton;
    private final BooleanSupplier aButton;
    private final PidController pidController;

    public TeleopElevatorCommand(Elevator elevator, BooleanSupplier aButton, BooleanSupplier bButton) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.aButton = aButton;
        this.bButton = bButton;
        this.pidController = elevator.pidController;

        

    }

    @Override
    public void execute() {

        boolean abutton = this.aButton.getAsBoolean();
        boolean bbutton = this.bButton.getAsBoolean();

        if(abutton) {
            pidController.setTargetPoint(20);
        } else if(bbutton) {
            pidController.setTargetPoint(30);
        }

        pidController.update();

        elevator.setMotors(pidController.GetForce());
    }
}
