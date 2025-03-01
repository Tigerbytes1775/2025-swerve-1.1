package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PidController;
import frc.robot.subsystems.Elevator;

public class TeleopElevatorCommand extends Command{
    
    private final Elevator elevator;
    private final BooleanSupplier bButton;
    private final BooleanSupplier aButton;
    private final BooleanSupplier xButton;
    private final BooleanSupplier yButton;

    public TeleopElevatorCommand(Elevator elevator, BooleanSupplier aButton, BooleanSupplier bButton, BooleanSupplier xButton, BooleanSupplier yButton) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;

        

    }

    @Override
    public void execute() {

        boolean abutton = this.aButton.getAsBoolean();
        boolean bbutton = this.bButton.getAsBoolean();
        boolean xbutton = this.xButton.getAsBoolean();
        boolean ybutton = this.yButton.getAsBoolean();

        if(abutton) {
            elevator.setTarget(0);
        } else if(bbutton) {
            elevator.setTarget(1);
        } else if(xbutton) {
            elevator.setTarget(2);
        } else if(ybutton) {
            elevator.setTarget(3);
        }

        elevator.update();

    }
}
