package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PathRunner;

public class TeleopPathCommand extends Command {

    private boolean commandRunning = false;

    private final PathRunner pathRunner;

    private final BooleanSupplier aButton;
    private final BooleanSupplier bButton;
    private final BooleanSupplier xButton;
    private final BooleanSupplier yButton;

    public TeleopPathCommand(PathRunner pathRunner, BooleanSupplier aButton, BooleanSupplier bButton, BooleanSupplier xButton, BooleanSupplier yButton) {
        addRequirements(pathRunner);

        this.pathRunner = pathRunner;

        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;
    
    }

    @Override
    public void execute() {
        boolean aButton = this.aButton.getAsBoolean();
        boolean bButton = this.bButton.getAsBoolean();
        boolean xButton = this.xButton.getAsBoolean();
        boolean yButton = this.yButton.getAsBoolean();

        boolean anyButtonPressed = aButton || bButton || xButton || yButton;



        if(!anyButtonPressed && commandRunning) {
            //pathRunner.stopPaths();
            commandRunning = false;

        }

        if(!commandRunning) {
            if(aButton) {
                pathRunner.goToFeeder();
                commandRunning = true;
            } 
        }
 
    }


}
