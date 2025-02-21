package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class TeleopAlgaeIntakeCommand extends Command{
    
    private final AlgaeIntake algaeIntake;
    private final BooleanSupplier rBumper;
    private final BooleanSupplier lBumper;

    public TeleopAlgaeIntakeCommand (AlgaeIntake algaeIntake, BooleanSupplier rBumper, BooleanSupplier lBumper) {

        addRequirements(algaeIntake);
        this.algaeIntake = algaeIntake;
        this.rBumper = rBumper;
        this.lBumper = lBumper;
 
    }

    @Override
    public void execute() {

        boolean rBumper = this.rBumper.getAsBoolean();
        boolean lBumper = this.lBumper.getAsBoolean();
        double power = 0;

        if(rBumper) {
            power = 1;
        } else if (lBumper) {
            power = -1;
        }

        algaeIntake.setMotors(power);
    }
}