// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class PivotSubsystem extends SubsystemBase {

    private final SparkMax pivotLeft;
    private final SparkMax pivotRight;

    public PivotSubsystem() {

        // Create motors
        pivotLeft = new SparkMax(IntakePivotConstants.PIVOT_LEFT_MOTOR_ID, MotorType.kBrushless);
        pivotRight = new SparkMax(IntakePivotConstants.PIVOT_RIGHT_MOTOR_ID, MotorType.kBrushless);

        // Left motor configuration
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .smartCurrentLimit(IntakePivotConstants.kPivotCurrentLimit)
            .inverted(IntakePivotConstants.kPivotLeftInverted)
            .idleMode(IntakePivotConstants.kPivotLeftIdleMode);

        // Right motor configuration
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .smartCurrentLimit(IntakePivotConstants.kPivotCurrentLimit)
            .inverted(IntakePivotConstants.kPivotRightInverted)
            .idleMode(IntakePivotConstants.kPivotRightIdleMode);

        pivotLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotUp() {
        pivotLeft.set(IntakePivotConstants.kPivotUpSpeed);
        pivotRight.set(IntakePivotConstants.kPivotUpSpeed);
    }

    public void pivotDown() {
        pivotLeft.set(IntakePivotConstants.kPivotDownSpeed);
        pivotRight.set(IntakePivotConstants.kPivotDownSpeed);
    }

    public void stopPivot() {
        pivotLeft.set(0);
        pivotRight.set(0);
    }

    @Override
    public void periodic() {
        // Runs every robot loop
    }
}