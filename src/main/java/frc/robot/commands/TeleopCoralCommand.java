package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Coral;

public class TeleopCoralCommand extends Command{
    
    private final Coral coral;
    private DoubleSupplier leftTrigger;
    private DoubleSupplier rightTrigger;
    private final double deadband = 0.1;

    public TeleopCoralCommand(Coral coral, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        addRequirements(coral);
        this.coral = coral;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        
    }

    @Override
    public void execute() {

        double leftTrigger = this.leftTrigger.getAsDouble();
        double rightTrigger = this.rightTrigger.getAsDouble();

        
        double coralPower = leftTrigger > deadband ? 0.8 : rightTrigger > deadband ? -0.8 : 0;

        coral.setMotors(coralPower);
    }
    
}
