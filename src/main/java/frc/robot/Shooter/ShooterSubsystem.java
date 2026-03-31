package frc.robot.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterLeft;
    private final TalonFX shooterRight;
    public ShooterSubsystem() {

        shooterLeft = new TalonFX(ShooterConstants.shooterLEFT_ID, "6348 Horus CANivore");
        shooterRight = new TalonFX(ShooterConstants.shooterRIGHT_ID, "6348 Horus CANivore");
        configureMotor(shooterLeft, ShooterConstants.shooterLEFT_INVERTED);
        configureMotor(shooterRight, ShooterConstants.shooterRIGHT_INVERTED); 
    }

    private void configureMotor(TalonFX motor, boolean inverted) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        // PID GAINS NECESARIOS PARA VELOCITY
        config.Slot0.kP = 0.25;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        motor.getConfigurator().apply(config);
    }


    // SHOOTER A VELOCIDAD (RPS)
    public void runShooterV1(double percent) {
        shooterLeft.setControl(new DutyCycleOut(0.41));
        shooterRight.setControl(new DutyCycleOut(0.41));
    }

    public void runShooterV2(double percent) {
        shooterLeft.setControl(new DutyCycleOut(0.345));
        shooterRight.setControl(new DutyCycleOut(0.345));
    }

    public void shoot (double DutyCycleOut){
        this.shooterLeft.setControl(new DutyCycleOut(DutyCycleOut));
        this.shooterRight.setControl(new DutyCycleOut(DutyCycleOut));
    }   

    // SHOOTER A PORCENTAJE ESPECIAL PARA LANZAR DE MEDIA CANCHA
    public void runShooterMidField(double percent) {
        shooterLeft.setControl(new DutyCycleOut(0.7));
        shooterRight.setControl(new DutyCycleOut(0.7));
    }

    

    

    // APAGA TODO
    public void stopAll() {
        shooterLeft.setControl(new DutyCycleOut(0));
        shooterRight.setControl(new DutyCycleOut(0));
       }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("RPM Hood Velocity Right", shooterRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("RPM Hood Velocity Left", shooterLeft.getVelocity().getValueAsDouble());
    }
}

       