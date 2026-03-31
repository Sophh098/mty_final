package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Shooter.ShooterSubsystem;

public class RunMiddleShooterCmd extends Command {

        ShooterSubsystem shooterSubsystem;

        public RunMiddleShooterCmd(ShooterSubsystem shooterSubsystem) {
                this.shooterSubsystem = shooterSubsystem;
                addRequirements(shooterSubsystem);
        }

        @Override
        public void initialize() {
                // TODO Auto-generated method stub
                
        }

        @Override
        public void execute() {
                
        shooterSubsystem.shoot(ShooterConstants.shooterMiddleDutyCycle);
                
        }

        @Override
        public void end(boolean interrupted) {

        shooterSubsystem.stopAll();
                
        }

        @Override
        public boolean isFinished() {
                // TODO Auto-generated method stub
                return false;
        }
    
}
