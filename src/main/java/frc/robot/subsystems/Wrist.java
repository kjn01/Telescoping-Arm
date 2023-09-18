// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  private CANSparkMax pivotMotor;
  private CANSparkMax rollerMotor;
  private AbsoluteEncoder pivotEncoder;

  private PIDController pivotController;

  public Wrist() {

    pivotMotor = new CANSparkMax(WristConstants.kPivotCanId, MotorType.kBrushless);
    rollerMotor = new CANSparkMax(WristConstants.kRollerCanId, MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    rollerMotor.setIdleMode(IdleMode.kCoast);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    pivotEncoder.setPositionConversionFactor(WristConstants.kPivotPositionConversionFactor);
    pivotEncoder.setZeroOffset(WristConstants.kPivotZeroOffset);

    pivotController = new PIDController(0, 0, 0);

    pivotController.setP(WristConstants.kPivotP);

    pivotMotor.burnFlash();
    rollerMotor.burnFlash();

  }

  public double getWristAngle() {
    return (pivotEncoder.getPosition() - WristConstants.kPivotKinematicOffset) / WristConstants.kPivotGearRatio;
  }

  public void setTargetWristAngle(double targetAngle) {
    pivotController.setSetpoint(targetAngle);
    SmartDashboard.putNumber("Target Wrist Angle", targetAngle);
  }

  public void setRollerInput() {
    rollerMotor.setVoltage(WristConstants.kRollerInputVoltage);
  }

  public void setRollerOutput() {
    rollerMotor.setVoltage(WristConstants.kRollerOutputVoltage);
  }

  public void setCalculatedWristVoltage() {
    double voltage = pivotController.calculate(getWristAngle());

    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("Wrist Voltage", voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCalculatedWristVoltage();

    SmartDashboard.putNumber("Current Wrist Angle", getWristAngle());
  }
}
