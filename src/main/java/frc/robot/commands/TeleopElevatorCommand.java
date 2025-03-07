package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PidController;
import frc.robot.subsystems.Elevator;

public class TeleopElevatorCommand extends Command{
    
    private final Elevator elevator;
    private final DoubleSupplier rightY;
    //private final BooleanSupplier bButton;
    //private final BooleanSupplier aButton;
    //private final BooleanSupplier xButton;
    //private final BooleanSupplier yButton;

    public TeleopElevatorCommand(Elevator elevator, DoubleSupplier rightY) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.rightY = rightY;

        

        

    }

    @Override
    public void execute() {

        double rightY = this.rightY.getAsDouble();
        double power = 0;
        if(rightY > 0.1 || rightY < -0.1) {
            power = rightY;
        }
        elevator.setMotors(power);

        /* 
        boolean abutton = this.aButton.getAsBoolean();
        boolean bbutton = this.bButton.getAsBoolean();
        boolean xbutton = this.xButton.getAsBoolean();
        boolean ybutton = this.yButton.getAsBoolean();

        if(abutton) {
            elevator.setTarget(0);
        } else if(bbutton) {
            elevator.setTarget(0.25);
        } else if(xbutton) {
            elevator.setTarget(0.5);
        } else if(ybutton) {
            elevator.setTarget(0.75);
        }

        elevator.update();*/

        

    }
}
