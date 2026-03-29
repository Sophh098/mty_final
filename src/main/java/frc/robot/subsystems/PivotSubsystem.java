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

    private final SparkMax pivot;
    public PivotSubsystem() {

        // Create motors
        pivot = new SparkMax(IntakePivotConstants.PIVOT_RIGHT_MOTOR_ID, MotorType.kBrushless);
      

        // Left motor configuration
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .smartCurrentLimit(IntakePivotConstants.kPivotCurrentLimit)
            .inverted(IntakePivotConstants.kPivotRightInverted)
            .idleMode(IntakePivotConstants.kPivotRightIdleMode);

        // Right motor configuration
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .smartCurrentLimit(IntakePivotConstants.kPivotCurrentLimit)
            .inverted(IntakePivotConstants.kPivotRightInverted)
            .idleMode(IntakePivotConstants.kPivotRightIdleMode);

        pivot.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotUp() {
       pivot.set(IntakePivotConstants.kPivotUpSpeed);
    }

    public void pivotDown() {
       pivot.set(IntakePivotConstants.kPivotDownSpeed);
    }

    public void stopPivot() {
        pivot.set(0);
    }

    @Override
    public void periodic() {
        // Runs every robot loop
    }
}
    