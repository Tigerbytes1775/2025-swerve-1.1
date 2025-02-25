package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class VisionCommand extends Command {
    
    private final Vision vision;

    public VisionCommand(Vision vision) {
        addRequirements(vision);
        this.vision = vision;
        //SmartDashboard.putBoolean("Vision Runs", true);
    }

    

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Vision Runs", true);
        vision.addVisionMeasurement();
    }

<<<<<<< HEAD
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    
=======
    @Override 
    public boolean runsWhenDisabled() {
        return true;
    }
>>>>>>> f5bf81d3fc68c9ec373898dcebcc3f8bc47eb6d7
        
          
}
