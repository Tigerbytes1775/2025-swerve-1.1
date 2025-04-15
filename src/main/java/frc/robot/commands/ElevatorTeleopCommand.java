package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorTeleopCommand extends Command{

    private final Elevator elevator;
    private final XboxController controller;

    public ElevatorTeleopCommand(Elevator elevator, XboxController controller) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.controller = controller;

    } 

    @Override
    public void initialize() {
        elevator.zeroEncoder();
        elevator.pidEnabled = false;
    }

    @Override
    public void execute() {
        double rightY = MathUtil.applyDeadband(controller.getRightY(), 0.25);

        boolean aButton = controller.getAButtonPressed();
        boolean bButton = controller.getBButtonPressed();
        boolean xButton = controller.getXButtonPressed();
        boolean yButton = controller.getYButtonPressed();

        boolean anyPidButton = aButton || bButton || xButton || yButton;

        int dpad = controller.getPOV();

        if(dpad == 180) {
            elevator.zeroEncoder();
        }

        elevator.pidEnabled = 
            rightY != 0 ?
            false : anyPidButton ?
            true : elevator.pidEnabled;
            

        

        if(elevator.pidEnabled) {
            
            if(anyPidButton) {
                elevator.setTarget(
                    aButton ? 
                    elevator.L1Height : bButton ?
                    elevator.L2Height : xButton ?
                    elevator.L3Height : elevator.L4Height
                );
            }
            
            elevator.update();
        } else {
            elevator.setMotors(rightY);

            
        }
        
    };
}
    

