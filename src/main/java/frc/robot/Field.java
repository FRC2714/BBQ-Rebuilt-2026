package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants.Hub;

/** Alliance-aware field utilities. */
public class Field {
  /** True if the robot is on the red alliance. */
  public static boolean isRed() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  /** Returns the 3D position of the current alliance's hub. */
  public static Translation3d getAllianceHub() {
    return isRed() ? Hub.oppTopCenterPoint : Hub.topCenterPoint;
  }
}
