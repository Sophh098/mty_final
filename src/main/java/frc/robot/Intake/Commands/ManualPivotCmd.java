package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.PivotSubsystem;

public class ManualPivotCmd extends Command {
    
    PivotSubsystem pivotSubsystem;
    boolean isCounterclockwise;

    public ManualPivotCmd(PivotSubsystem pivotSubsystem, boolean isCounterclockwise) {
            this.pivotSubsystem = pivotSubsystem;
            this.isCounterclockwise = isCounterclockwise;
            addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
            // TODO Auto-generated method stub
            
    }

        @Override
        public void execute() {
                if (isCounterclockwise) {
                        pivotSubsystem.pivotDown();
                } else {
                        pivotSubsystem.pivotUp();
                }
        }       

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stopPivot();
            
    }

    @Override
    public boolean isFinished() {
            // TODO Auto-generated method stub
            return false;
    }

}
