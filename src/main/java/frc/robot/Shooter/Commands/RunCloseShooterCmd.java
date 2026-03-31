package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Shooter.ShooterSubsystem;

public class RunCloseShooterCmd extends Command {

        ShooterSubsystem shooterSubsystem;

        public RunCloseShooterCmd(ShooterSubsystem shooterSubsystem) {
                this.shooterSubsystem = shooterSubsystem;
                addRequirements(shooterSubsystem);
        }

        @Override
        public void initialize() {
                // TODO Auto-generated method stub
                
        }

        @Override
        public void execute() {
                
        shooterSubsystem.shoot(ShooterConstants.shooterCloseDutyCycle);
                
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
