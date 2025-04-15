package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class CoralAutoCommand extends Command {

    private final Coral coral;
    private final boolean out;
    private final double power;
    private final double commandTime;
    private final Timer timer = new Timer();
    
    public CoralAutoCommand(Coral coral, boolean out, double power, double commandTime) {
        addRequirements(coral);
        this.coral = coral;
        this.out = out;
        this.power = power;
        this.commandTime = commandTime;
    }


    @Override 
    public void initialize() {
        timer.reset();
        timer.start();
        if(out) {
            coral.setOutMotor(power);
        } else {
            coral.setInMotor(power);
        }
    }

    @Override
    public void execute() {

    }

    @Override 
    public boolean isFinished() {
        return timer.get() >= commandTime;
    }

    @Override 
    public void end(boolean interrupted) {
        if(out) {
            coral.setOutMotor(0);
        } else {
            coral.setInMotor(0);
        }
    }
}
