package frc.robot.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HopperIndexer.HopperIndexerSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.PivotSubsystem;
import frc.robot.Shooter.ShooterSubsystem;

public class ShootAutoCmd extends SequentialCommandGroup {

    public ShootAutoCmd(
        frc.robot.Drive.CommandSwerveDrivetrain drivetrain,
        PivotSubsystem intakePivot,
        IntakeSubsystem intakeSubsystem,
        HopperIndexerSubsystem hopperIndexerSubsystem,
        ShooterSubsystem shooterSubsystem,
        double shooterPercentOutput,
        double feedTimeSeconds
    ) {
        addCommands(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            Commands.runOnce(intakePivot::deployed, intakePivot),

            Commands.runOnce(
                () -> shooterSubsystem.shoot(shooterPercentOutput),
                shooterSubsystem
            ),

            Commands.waitSeconds(2.5),

            Commands.parallel(
                Commands.run(
                    () -> hopperIndexerSubsystem.runIndexer(0.6),
                    hopperIndexerSubsystem
                ),
                Commands.run(
                    () -> hopperIndexerSubsystem.runHopper(0.6),
                    hopperIndexerSubsystem
                ),
                Commands.run(
                    intakeSubsystem::RunIntake,
                    intakeSubsystem
                )
            ).withTimeout(feedTimeSeconds),

            Commands.runOnce(shooterSubsystem::stopAll, shooterSubsystem),
            Commands.runOnce(intakeSubsystem::stopRoller, intakeSubsystem),
            Commands.runOnce(hopperIndexerSubsystem::StopIt, hopperIndexerSubsystem)
        );
    }
}