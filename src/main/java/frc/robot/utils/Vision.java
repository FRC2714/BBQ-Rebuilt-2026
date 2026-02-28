package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.List;

/** Vision utilities for converting Limelight detections to field poses. */
public class Vision {
  /** Returns 3D poses of all AprilTags currently detected by the given Limelight. */
  public static Pose3d[] getCameraTargetPoses3d(String limelightName) {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
    List<Pose3d> poses = new ArrayList<>();

    for (RawFiducial fiducial : fiducials) {
      AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(fiducial.id).ifPresent(poses::add);
    }

    return poses.toArray(new Pose3d[0]);
  }
}
