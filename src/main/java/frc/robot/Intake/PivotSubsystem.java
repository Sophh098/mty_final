package frc.robot.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class PivotSubsystem extends SubsystemBase {

    private static final double maximumAutomaticOutput = 0.40;
    private static final double positionTolerance = 1.0;

    public static final double homePosition = 0.0;
    public static final double deployedPosition = -43.3;

    private final SparkMax pivotMotor;
    private final PIDController pivotPidController;
    public final RelativeEncoder pivotEncoder;

    private boolean manualControlEnabled;
    private double targetPosition;

    public PivotSubsystem() {
        pivotMotor = new SparkMax(
            IntakePivotConstants.PIVOT_RIGHT_MOTOR_ID,
            MotorType.kBrushless
        );

        pivotPidController = new PIDController(0.005, 0.0, 0.0);
        pivotPidController.setTolerance(positionTolerance);

        SparkMaxConfig pivotMotorConfiguration = new SparkMaxConfig();
        pivotMotorConfiguration
            .smartCurrentLimit(IntakePivotConstants.kPivotCurrentLimit)
            .inverted(IntakePivotConstants.kPivotRightInverted)
            .idleMode(IntakePivotConstants.kPivotRightIdleMode);

        AlternateEncoderConfig alternateEncoderConfiguration = new AlternateEncoderConfig();
        alternateEncoderConfiguration.setSparkMaxDataPortConfig();

        pivotMotorConfiguration.apply(alternateEncoderConfiguration);

        pivotMotor.configure(
            pivotMotorConfiguration,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        pivotEncoder = pivotMotor.getAlternateEncoder();
        pivotEncoder.setPosition(0.0);

        manualControlEnabled = false;
        targetPosition = pivotEncoder.getPosition();
    }

    public void pivotUp() {
        manualControlEnabled = true;
        pivotMotor.set(IntakePivotConstants.kPivotUpSpeed);
    }

    public void pivotDown() {
        manualControlEnabled = true;
        pivotMotor.set(IntakePivotConstants.kPivotDownSpeed);
    }

    public void stopPivot() {
        holdCurrentPosition();
    }

    //Automatic start position
        public void home() {
        setTargetPosition(homePosition);
    }

    public void deployed() {
        setTargetPosition(deployedPosition);
    }

    public void setTargetPosition(double newTargetPosition) {
        targetPosition = newTargetPosition;
        manualControlEnabled = false;
        pivotPidController.reset();
    }

    public void holdCurrentPosition() {
        targetPosition = pivotEncoder.getPosition();
        manualControlEnabled = false;
        pivotPidController.reset();
    }

    public void zeroEncoder() {
        pivotEncoder.setPosition(0.0);
        targetPosition = 0.0;
        pivotPidController.reset();
    }

    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public boolean isAtSetpoint() {
        return pivotPidController.atSetpoint();
    }

   

    @Override
    public void periodic() {
        double currentPosition = pivotEncoder.getPosition();

        if (!manualControlEnabled) {
            double automaticOutput = pivotPidController.calculate(currentPosition, targetPosition);
            double limitedAutomaticOutput = MathUtil.clamp(
                automaticOutput,
                -maximumAutomaticOutput,
                maximumAutomaticOutput
            );

            pivotMotor.set(limitedAutomaticOutput);
        }

        SmartDashboard.putNumber("Pivot/Position", currentPosition);
        SmartDashboard.putNumber("Pivot/TargetPosition", targetPosition);
        SmartDashboard.putNumber("Pivot/Error", pivotPidController.getError());
        SmartDashboard.putBoolean("Pivot/ManualControlEnabled", manualControlEnabled);
        SmartDashboard.putBoolean("Pivot/AtSetpoint", pivotPidController.atSetpoint());
    }
}