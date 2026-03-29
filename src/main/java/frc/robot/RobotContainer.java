// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController driver = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); 
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final PivotSubsystem intakePivot = new PivotSubsystem();


    
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        drivetrain.registerTelemetry(logger::telemeterize);
    
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Left Trigger = Indexer + Hopper
        joystick.leftTrigger().whileTrue(Commands.run(() -> shooterSubsystem.runIndexerHopper(0.6, 0.6), shooterSubsystem)).onFalse(Commands.runOnce(() -> shooterSubsystem.stopAll(), shooterSubsystem));

        //Right Trigger = runShooterMidField
        joystick.rightTrigger().whileTrue(Commands.run(() -> shooterSubsystem.runShooterMidField(0.7), shooterSubsystem)).onFalse(Commands.runOnce(() -> shooterSubsystem.stopAll(), shooterSubsystem));

    ///-----------------------ADDONS-----------------------

        //  A Button = intake
        driver.a().whileTrue(Commands.run(() -> intake.RunIntake(), intake)).onFalse(Commands.runOnce(() -> intake.stopRoller(), intake));

        // B Button = outtake
        driver.b().whileTrue(Commands.run(() -> intake.RunOuttake(), intake)).onFalse(Commands.runOnce(() -> intake.stopRoller(), intake));

        // Right Bumper = Pivot Up
        driver.rightBumper().whileTrue(Commands.run(() -> intakePivot.pivotUp(), intakePivot)).onFalse( Commands.runOnce(() -> intakePivot.stopPivot(), intakePivot));

        // Left Bumper = Pivot Down
        driver.leftBumper().whileTrue(Commands.run(() -> intakePivot.pivotDown(), intakePivot)).onFalse( Commands.runOnce(() -> intakePivot.stopPivot(), intakePivot));

       // Y= medida rara de jose pablo
        driver.y().whileTrue(Commands.run(() -> shooterSubsystem.runShooterV2(0.375), shooterSubsystem)).onFalse(Commands.runOnce(() -> shooterSubsystem.stopAll(), shooterSubsystem));

       // Right Triggr = ShooterRPS
        driver.rightTrigger().whileTrue(Commands.run(() -> shooterSubsystem.runShooterV1(0.43), shooterSubsystem)).onFalse(Commands.runOnce(() -> shooterSubsystem.stopAll(), shooterSubsystem));

        //Left Trigger = Indexer + Hopper
        driver.leftTrigger().whileTrue(Commands.run(() -> shooterSubsystem.runIndexerHopper(0.6, 0.6), shooterSubsystem)).onFalse(Commands.runOnce(() -> shooterSubsystem.stopAll(), shooterSubsystem));


    // X button = indexer outtake
driver.x().whileTrue(new StartEndCommand( () -> shooterSubsystem.outtakeIndexer(), () -> shooterSubsystem.runIndexer(0), shooterSubsystem));




    }


    public Command getAutonomousCommand() {
    final var idle = new SwerveRequest.Idle();

    return Commands.sequence(

        // Reset heading
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

        // Spin shooter
       //  Commands.runOnce(() -> {
       //     double targetRPS = ShooterConstants.kShooterRPM / 60.0;
       //     shooterSubsystem.runShooterRPS(targetRPS);
        //}), 

        // Wait for shooter to reach speed
        Commands.waitSeconds(2.5),

        // Feed the note
        Commands.run(() -> shooterSubsystem.runIndexer(0.6))
            .withTimeout(6.0),

        // Stop shooter and indexer
        Commands.runOnce(() -> shooterSubsystem.stopAll()),
        
        // Idle rest of auton
        drivetrain.applyRequest(() -> idle)
    );

    }
}
