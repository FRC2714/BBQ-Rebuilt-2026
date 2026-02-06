package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants.Hub;

public class Field {
  public static boolean isRed() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public static Translation3d getAllianceHub() {
    return isRed() ? Hub.oppTopCenterPoint : Hub.topCenterPoint;
  }
}
