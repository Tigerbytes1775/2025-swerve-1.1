package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class SwerveTeleopCommand extends Command {

    private final BooleanSupplier dPad;
    private final SwerveSubsystem drivebase;
    private final SwerveDrive swerveDrive;
    private final SwerveInputStream driveAngularVelocity;
    private final SwerveInputStream driveDirectAngle;


    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, SwerveDrive swerveDrive, SwerveInputStream driveAngularVelocity, SwerveInputStream driveDirectAngle, BooleanSupplier dPad) {
        addRequirements(swerveSubsystem);
        this.drivebase = swerveSubsystem;
        this.swerveDrive = swerveDrive;
        this.driveAngularVelocity = driveAngularVelocity;
        this.driveDirectAngle = driveDirectAngle;
        this.dPad = dPad;
        
            
        
    }


    
    @Override
    public void execute() {
        if(dPad.getAsBoolean()) {
            drivebase.zeroGyro();
        } 
        drivebase.driveFieldOriented(drivebase.getRobotVelocity());
        //drivebase.driveFieldOriented(driveDirectAngle);
        //drivebase.driveFieldOriented(driveAngularVelocity);
        
    }
    
}
