package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeSubsystem;

public class RunIntakeCmd extends Command {

        IntakeSubsystem intakeSubsystem;

        boolean inverted;

        double DutyCycleOut;

        public RunIntakeCmd(IntakeSubsystem intakeSubsystem, boolean inverted, double DutyCycleOut) {
                this.intakeSubsystem = intakeSubsystem;
                this.DutyCycleOut= DutyCycleOut;
                this.inverted = inverted;
                addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
                // TODO Auto-generated method stub
                
        }

        @Override
        public void execute() {

                DutyCycleOut *= inverted ? -1 : 1;

                intakeSubsystem.RunIntake(DutyCycleOut);
                
        }

        @Override
        public void end(boolean interrupted) {
               intakeSubsystem.stopRoller();
                
        }

        @Override
        public boolean isFinished() {
                // TODO Auto-generated method stub
                return false;
        }
    
}
