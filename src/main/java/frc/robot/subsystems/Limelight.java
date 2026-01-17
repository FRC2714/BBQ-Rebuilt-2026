// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase {
  
  private String m_limelightName;

  public Limelight(String limelightName) {
    m_limelightName = limelightName;
  }

  //need way to find the best pipeline to swithc to when align
 public void setPipeline(){
    LimelightHelpers.setPipelineIndex(m_limelightName, 0);
    double bestID1 = bestTarget();

    LimelightHelpers.setPipelineIndex(m_limelightName, 1);
    double bestID2 = bestTarget();

    LimelightHelpers.setPipelineIndex(m_limelightName, 2);
    double bestID3 = bestTarget();

    if(bestID1 >= bestID2 && bestID1 >= bestID3){
      LimelightHelpers.setPipelineIndex(m_limelightName, 0);
    } else if (bestID2 >= bestID1 && bestID2 >= bestID3){
      LimelightHelpers.setPipelineIndex(m_limelightName, 1);
    } else {
      LimelightHelpers.setPipelineIndex(m_limelightName, 2);
    }
 }
  public int getTargetID() {
    return (int) LimelightHelpers.getFiducialID(m_limelightName);
  }

  public boolean isTargetVisible() {
    return LimelightHelpers.getTV(m_limelightName);
  }

  public double bestTarget(){
    return LimelightHelpers.getTA(m_limelightName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("bets tag", getTargetID());
  }
}
