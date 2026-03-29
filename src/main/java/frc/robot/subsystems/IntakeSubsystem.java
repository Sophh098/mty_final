package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
   private final TalonFX roller;

    public IntakeSubsystem() {
        roller = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, "6348 Horus CANivore");
        configureMotor(roller, IntakeConstants.kRollerInverted);
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

   public void RunIntake(){
    roller.set(IntakeConstants.kIntakeSpeed);
   }

   public void RunOuttake(){
    roller.set(IntakeConstants.kOuttakeSpeed);
   }

    public void stopRoller(){
     roller.set(0);
    }
    
     @Override
     public void periodic() {
          // Runs every robot loop
     }
}