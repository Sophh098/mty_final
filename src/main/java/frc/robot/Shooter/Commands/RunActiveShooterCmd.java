package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.Shooter.ShooterMath;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Vision.VisionSubsystem;

/**
 * Comando que corre el flywheel mientras el boton se mantiene presionado.
 *
 * <h2>Logica de seleccion de DutyCycle</h2>
 * <ul>
 *   <li><b>Vision con tags + tiro valido</b>: RPM de la lookup table / 5800 = DutyCycle.</li>
 *   <li><b>Sin vision o tiro invalido</b>: DutyCycle default ({@link ShooterConstants#kShooterSpeed}).</li>
 * </ul>
 *
 * <h2>Uso tipico en RobotContainer</h2>
 * <pre>{@code
 * shootButton.whileTrue(new RunActiveShooterCmd(
 *     shooterSubsystem,
 *     drivetrain,
 *     visionSubsystem
 * ));
 * }</pre>
 */
public class RunActiveShooterCmd extends Command {

    
}