package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorAutoCommand extends Command{

    private final Elevator elevator;
    private final double target;
    private final double commandTime;
    private final Timer timer = new Timer();

    public ElevatorAutoCommand(Elevator elevator, double target, double commandTime) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.target = target;
        this.commandTime = commandTime;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        elevator.setTarget(target);
    }
    @Override
    public void execute() {
        //System.out.println("Elevator Auto On");
        elevator.update();
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= commandTime;
    }

}
