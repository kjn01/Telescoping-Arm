// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import frc.robot.Constants.ArmConstants;
import frc.utils.ArmPreset;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmStateMachine extends SubsystemBase {
  /** Creates a new ArmStateMachine. */
  private Arm m_arm;

  private ArmScoreLevel armScoreLevel;
  private CargoType cargoType;

  public enum CargoType {
    CONE, CUBE
  }

  public enum ArmState {
    STOW, BACK, FRONT
  }

  public enum ArmScoreLevel {
    HIGH, MIDDLE, LOW, SHELF_INTAKE
  }

  Map<ArmScoreLevel, Command> backConeMap = Map.ofEntries(
    Map.entry(ArmScoreLevel.HIGH, m_arm.setPositionCommand(ArmConstants.kBackConeHighPosition)),
    Map.entry(ArmScoreLevel.MIDDLE, m_arm.setPositionCommand(ArmConstants.kBackConeMiddlePosition)),
    Map.entry(ArmScoreLevel.LOW, m_arm.setPositionCommand(ArmConstants.kBackConeLowPosition))
  );

  Map<ArmScoreLevel, Command> backCubeMap = Map.ofEntries(
    Map.entry(ArmScoreLevel.HIGH, m_arm.setPositionCommand(ArmConstants.kBackCubeHighPosition)),
    Map.entry(ArmScoreLevel.MIDDLE, m_arm.setPositionCommand(ArmConstants.kBackCubeMiddlePosition)),
    Map.entry(ArmScoreLevel.LOW, m_arm.setPositionCommand(ArmConstants.kBackCubeLowPosition))
  );

  Map<ArmScoreLevel, Command> frontIntakeMap = Map.ofEntries(
    Map.entry(ArmScoreLevel.SHELF_INTAKE, m_arm.setPositionCommand(ArmConstants.kFrontShelfIntakePosition)),
    Map.entry(ArmScoreLevel.LOW, m_arm.setPositionCommand(ArmConstants.kFrontFloorIntakePosition))
  );

  public ArmStateMachine(Arm m_arm) {

    this.m_arm = m_arm;

  }

  public InstantCommand setArmScoreLevelCommand(ArmScoreLevel armScoreLevel) {
    SmartDashboard.putString("Arm Score Level", armScoreLevel.toString());
    return new InstantCommand(() -> this.armScoreLevel = armScoreLevel);
  }

  public InstantCommand setCargoTypeCommand(CargoType cargoType) {
    SmartDashboard.putString("Arm Score Level", armScoreLevel.toString());
    return new InstantCommand(() -> this.cargoType = cargoType);
  }

  // public Command getArmCommand(ArmScoreLevel armScoreLevel, CargoType cargoType) {
  //   SelectCommand backConeSelectCommand = new SelectCommand(backConeMap, () -> armScoreLevel);
  //   return backConeSelectCommand;
  // }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
