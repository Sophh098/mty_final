package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.PivotSubsystem;

public class DeployIntakeCmd extends Command {
    
    PivotSubsystem pivotSubsystem;

    public DeployIntakeCmd(PivotSubsystem pivotSubsystem) {
            this.pivotSubsystem = pivotSubsystem;
            addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
           pivotSubsystem.deployed();;
            
    }
    
}
