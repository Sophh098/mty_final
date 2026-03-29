package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterLeft;
    private final TalonFX shooterRight;
    private final TalonFX shooterIndexer;
    private final TalonFX shooterHopper;

    public ShooterSubsystem() {

        shooterLeft = new TalonFX(ShooterConstants.shooterLEFT_ID, "6348 Horus CANivore");
        shooterRight = new TalonFX(ShooterConstants.shooterRIGHT_ID, "6348 Horus CANivore");
        shooterIndexer = new TalonFX(ShooterConstants.shooterINDEXER_ID, "6348 Horus CANivore");
        shooterHopper = new TalonFX(ShooterConstants.shooterHOPPER_ID, "6348 Horus CANivore");
        configureMotor(shooterLeft, ShooterConstants.shooterLEFT_INVERTED);
        configureMotor(shooterRight, ShooterConstants.shooterRIGHT_INVERTED);
        configureMotor(shooterIndexer, ShooterConstants.shooterINDEXER_INVERTED);
        configureMotor(shooterHopper, ShooterConstants.shooterHOPPER_INVERTED);
    }

    private void configureMotor(TalonFX motor, boolean inverted) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        // PID GAINS NECESARIOS PARA VELOCITY
        config.Slot0.kP = 0.12;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        motor.getConfigurator().apply(config);
    }


    // SHOOTER A VELOCIDAD (RPS)
    public void runShooterV1(double percent) {
        shooterLeft.setControl(new DutyCycleOut(0.43));
        shooterRight.setControl(new DutyCycleOut(0.43));
    }

    public void runShooterV2(double percent) {
        shooterLeft.setControl(new DutyCycleOut(0.375));
        shooterRight.setControl(new DutyCycleOut(0.375));
    }



    // SHOOTER A PORCENTAJE ESPECIAL PARA LANZAR DE MEDIA CANCHA
    public void runShooterMidField(double percent) {
        shooterLeft.setControl(new DutyCycleOut(0.7));
        shooterRight.setControl(new DutyCycleOut(0.7));
    }

    // INDEXER
    public void runIndexer(double percent) {
        shooterIndexer.setControl(new DutyCycleOut(percent));
    }

    //INDEXER + HOPPER
    public void runIndexerHopper(double indexerPercent, double hopperPercent) {
        shooterIndexer.setControl(new DutyCycleOut(indexerPercent));
        shooterHopper.setControl(new DutyCycleOut(hopperPercent));
    }

    // INDEXER
    public void outtakeIndexer() {
    shooterIndexer.setControl(new DutyCycleOut(-0.5));
    }

    // HOPPER
    public void runHopper(){
        shooterHopper.setControl(new DutyCycleOut(0.6));
    }

    

    // APAGA TODO
    public void stopAll() {
        shooterLeft.setControl(new DutyCycleOut(0));
        shooterRight.setControl(new DutyCycleOut(0));
        shooterIndexer.setControl(new DutyCycleOut(0));
        shooterHopper.setControl(new DutyCycleOut(0));
    }

    @Override
    public void periodic() {

    }
}

       