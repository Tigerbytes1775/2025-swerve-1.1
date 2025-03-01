package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {

    private final BooleanSupplier dPad;
    private final SwerveSubsystem swerveSubsystem;

    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, BooleanSupplier dPad) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.dPad = dPad;
    }

    @Override
    public void execute() {
        if(dPad.getAsBoolean()) {
            swerveSubsystem.zeroGyro();
        } 
    }
    
}
