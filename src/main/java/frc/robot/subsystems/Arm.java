// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;
import frc.utils.ArmPreset;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Shoulder. */

  private CANSparkMax shoulderMotor;
  private CANSparkMax telescopeMotor;
  private AbsoluteEncoder shoulderEncoder;
  private RelativeEncoder telescopeEncoder;

  private ProfiledPIDController shoulderController;
  private ProfiledPIDController telescopeController;
  private ArmFeedforward shoulderFeedforward;
  private ElevatorFeedforward telescopeFeedforward;

  public Arm() {
    shoulderMotor = new CANSparkMax(ArmConstants.kShoulderCanId, MotorType.kBrushless);
    telescopeMotor = new CANSparkMax(ArmConstants.kTelescopeCanId, MotorType.kBrushless);
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    telescopeMotor.setIdleMode(IdleMode.kBrake);

    shoulderEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    telescopeEncoder = telescopeMotor.getEncoder();
    shoulderEncoder.setPositionConversionFactor(ArmConstants.kShoulderPositionConversionFactor);
    shoulderEncoder.setZeroOffset(ArmConstants.kShoulderZeroOffset);


    shoulderMotor.burnFlash();
    telescopeMotor.burnFlash();


    shoulderController = new ProfiledPIDController(0, 0, 0, ArmConstants.kShoulderConstraints);
    shoulderController.setP(ArmConstants.kShoulderP);
    shoulderFeedforward = new ArmFeedforward(0, 0, 0); // TBD

    telescopeController = new ProfiledPIDController(0, 0, 0, ArmConstants.kTelescopeConstraints);
    telescopeController.setP(ArmConstants.kTelescopeP);
    telescopeFeedforward = new ElevatorFeedforward(0, 0, 0); // TBD

    shoulderController.setTolerance(ArmConstants.shoulderPositionTolerance);
    telescopeController.setTolerance(ArmConstants.telescopePositionTolerance);
  }

  public double getShoulderAngle() {
    return (shoulderEncoder.getPosition() - ArmConstants.kShoulderKinematicOffset) / ArmConstants.kShoulderGearRatio;
  }

  public void setTargetShoulderAngle(double targetAngle) {
    shoulderController.setGoal(new State(targetAngle, 0));
    SmartDashboard.putNumber("Shoulder Target Angle", targetAngle);
  }

  public double getTelescopePosition() {
    return telescopeEncoder.getPosition();
  }

  public void setTargetTelescopePosition(double targetPosition) {
    telescopeController.setGoal(new State(targetPosition, 0));
    SmartDashboard.putNumber("Telescope Target Position", targetPosition);
  }

  private void setCalculatedShoulderVoltage() {
    double voltage = shoulderController.calculate(getShoulderAngle()) + shoulderFeedforward.calculate(getTelescopePosition(), 0);

    shoulderMotor.setVoltage(voltage);

    SmartDashboard.putNumber("Shoulder Voltage", voltage);
  }

  private void setCalculatedTelescopeVoltage() {
    double voltage = telescopeController.calculate(getTelescopePosition()) + telescopeFeedforward.calculate(getTelescopePosition(), 0);

    telescopeMotor.setVoltage(voltage);

    SmartDashboard.putNumber("Telescope Voltage", voltage);
  }

  public void setPosition(ArmPreset armPreset) {
    setTargetShoulderAngle(armPreset.shoulderAngle);
    setTargetTelescopePosition(armPreset.telescopePosition);
  }

  public InstantCommand setPositionCommand(ArmPreset armPreset) {
    return new InstantCommand(() -> setPosition(armPreset));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCalculatedShoulderVoltage();
    setCalculatedTelescopeVoltage();

    SmartDashboard.putNumber("Current Shoulder Angle", getShoulderAngle());
    SmartDashboard.putNumber("Current Telescope Position", getTelescopePosition());
  }
}
