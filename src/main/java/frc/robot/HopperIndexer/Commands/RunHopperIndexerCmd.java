package frc.robot.HopperIndexer.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HopperIndexer.HopperIndexerSubsystem;

public class RunHopperIndexerCmd extends Command {

        HopperIndexerSubsystem hopperIndexerSubsystem;

        boolean inverted;

        double DutyCycleOut;

        public RunHopperIndexerCmd( HopperIndexerSubsystem hopperIndexerSubsystem, boolean inverted, double DutyCycleOut){

                this.hopperIndexerSubsystem = hopperIndexerSubsystem;
                this.inverted = inverted;
                this.DutyCycleOut = DutyCycleOut;

                addRequirements(hopperIndexerSubsystem);

        }

    @Override
    public void initialize() {
            // TODO Auto-generated method stub
            
    }

    @Override
    public void execute() {

        DutyCycleOut *= inverted ? -1:1;

        hopperIndexerSubsystem.runIndexerHopper(DutyCycleOut, DutyCycleOut);
            
    }

    @Override
    public void end(boolean interrupted) {
            // TODO Auto-generated method stub
            
    }

    @Override
    public boolean isFinished() {
            // TODO Auto-generated method stub
            return false;
    }
    
}
