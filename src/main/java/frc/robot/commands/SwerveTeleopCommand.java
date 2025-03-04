package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class SwerveTeleopCommand extends Command {

    private final BooleanSupplier dPad;
    private final SwerveSubsystem driveBase;
    private final SwerveDrive swerveDrive;


    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, SwerveDrive swerveDrive, BooleanSupplier dPad) {
        addRequirements(swerveSubsystem);
        this.driveBase = swerveSubsystem;
        this.swerveDrive = swerveDrive;
        this.dPad = dPad;
        
    }

    @Override
    public void execute() {
        if(dPad.getAsBoolean()) {
            driveBase.zeroGyro();
        } 
        swerveDrive.driveFieldOriented(swerveDrive.getFieldVelocity());
    }
    
}
