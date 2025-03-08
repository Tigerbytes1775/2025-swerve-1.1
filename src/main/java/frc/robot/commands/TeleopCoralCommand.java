package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Coral;

public class TeleopCoralCommand extends Command{
    
    private final Coral coral;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;
    private final BooleanSupplier dPad;
    private final double deadband = 0.1;

    public TeleopCoralCommand(Coral coral, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, BooleanSupplier dPad) {
        addRequirements(coral);
        this.coral = coral;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.dPad = dPad;
        
    }

    @Override
    public void execute() {

        double leftTrigger = this.leftTrigger.getAsDouble();
        double rightTrigger = this.rightTrigger.getAsDouble();
        boolean dPad = this.dPad.getAsBoolean();

        
        double inPower = 0;
        double outPower = 0;
        
        if(leftTrigger > deadband) {
            outPower = 1;
        }

        if(rightTrigger > deadband) {
            inPower = 1;
        }

        if(dPad) {
            outPower = -0.2;
        }





        coral.setInMotor(inPower);
        coral.setOutMotor(outPower);
    }
    
}
