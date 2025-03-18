package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FieldConstants {
  public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
  public static final double FIELD_WIDTH = Units.inchesToMeters(317);

  /** Measured from the inside of starting line * */
  public static final double STARTING_LINE_X = Units.inchesToMeters(299.438);

  public static class Processor {
    public static final Pose2d CENTER_FACE =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d FAR_CAGE =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d MIDDLE_CAGE =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d CLOSE_CAGE =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    /** Measured from floor to bottom of cage * */
    public static final double DEEP_HEIGHT = Units.inchesToMeters(3.125);

    public static final double SHALLOW_HEIGHT = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d LEFT_CENTER_FACE =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d RIGHT_CENTER_FACE =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Offsets {
    public static final Pose3d CAMERA_OFFSET =
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Distance ADJUST_X = Inches.of(30.738);
    public static final Distance ADJUST_Y = Inches.of(6.469);
  }

  public static class Reef {
    public static final Translation2d CENTER =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

    /** Side of the reef to the inside of the reef zone line * */
    public static final double FACE_TO_ZONE_LINE = Units.inchesToMeters(12);

    /** Starting facing the driver station in clockwise order * */
    public static final List<Pose2d> CENTER_FACES =
        List.of(
            new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(180)),
            new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(120)),
            new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(60)),
            new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(0)),
            new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(-60)),
            new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(-120)));

    public static final List<Integer> CENTER_FACES_RED_IDS = List.of(7, 8, 9, 10, 11, 6);

    public static final List<Integer> CENTER_FACES_BLUE_IDS = List.of(18, 19, 20, 21, 22, 17);

    /** Starting at the right branch facing the driver station in clockwise * */
    public static Map<Direction, Map<Integer, Pose2d>> BRANCH_POSITIONS = new HashMap<>(2);

    public static final Pose2d ID17REEFRIGHTBRANCH =
        new Pose2d(3.3, 4.2, Rotation2d.fromDegrees(60));
    public static final Pose2d ID17REEFLEFTBRANCH =
        new Pose2d(3.2, 3.9, Rotation2d.fromDegrees(60));

    public static final Pose2d ID18REEFRIGHTBRANCH =
        new Pose2d(3.3, 4.2, Rotation2d.fromDegrees(0));
    public static final Pose2d ID18REEFLEFTBRANCH = new Pose2d(3.2, 4.2, Rotation2d.fromDegrees(0));

    public static final Pose2d ID19REEFRIGHTBRANCH =
        new Pose2d(3.7, 5, Rotation2d.fromDegrees(-60));
    public static final Pose2d ID19REEFLEFTBRANCH = new Pose2d(4, 5.1, Rotation2d.fromDegrees(-60));

    public static final Pose2d ID20REEFRIGHTTBRANCH =
        new Pose2d(5, 5.2, Rotation2d.fromDegrees(-120));
    public static final Pose2d ID20REEFLEFTBRANCH =
        new Pose2d(5.3, 5, Rotation2d.fromDegrees(-120));

    public static final Pose2d ID21REEFRIGHTTBRANCH =
        new Pose2d(5.8, 4.2, Rotation2d.fromDegrees(-180));
    public static final Pose2d ID21REEFLEFTBRANCH =
        new Pose2d(5.8, 4.2, Rotation2d.fromDegrees(-180));

    public static final Pose2d ID22REEFRIGHTBRANCH =
        new Pose2d(5.5, 2.6, Rotation2d.fromDegrees(120));
    public static final Pose2d ID22REEFLEFTBRANCH =
        new Pose2d(5.2, 2.3, Rotation2d.fromDegrees(120));

    static {
      HashMap<Integer, Pose2d> RIGHT_BRANCH_POSITIONS = new HashMap<>();
      HashMap<Integer, Pose2d> LEFT_BRANCH_POSITIONS = new HashMap<>();

      LEFT_BRANCH_POSITIONS.put(17, ID17REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(17, ID17REEFRIGHTBRANCH);
      LEFT_BRANCH_POSITIONS.put(6, ID17REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(6, ID17REEFRIGHTBRANCH);

      LEFT_BRANCH_POSITIONS.put(18, ID18REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(18, ID18REEFRIGHTBRANCH);
      LEFT_BRANCH_POSITIONS.put(7, ID18REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(7, ID18REEFRIGHTBRANCH);

      LEFT_BRANCH_POSITIONS.put(19, ID19REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(19, ID19REEFRIGHTBRANCH);
      LEFT_BRANCH_POSITIONS.put(8, ID19REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(8, ID19REEFRIGHTBRANCH);

      LEFT_BRANCH_POSITIONS.put(20, ID20REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(20, ID20REEFRIGHTTBRANCH);
      LEFT_BRANCH_POSITIONS.put(9, ID20REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(9, ID20REEFRIGHTTBRANCH);

      LEFT_BRANCH_POSITIONS.put(21, ID21REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(21, ID21REEFRIGHTTBRANCH);
      LEFT_BRANCH_POSITIONS.put(10, ID21REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(10, ID21REEFRIGHTTBRANCH);

      LEFT_BRANCH_POSITIONS.put(22, ID22REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(22, ID22REEFRIGHTBRANCH);
      LEFT_BRANCH_POSITIONS.put(11, ID22REEFLEFTBRANCH);
      RIGHT_BRANCH_POSITIONS.put(11, ID22REEFRIGHTBRANCH);

      BRANCH_POSITIONS.put(Direction.LEFT, LEFT_BRANCH_POSITIONS);
      BRANCH_POSITIONS.put(Direction.RIGHT, RIGHT_BRANCH_POSITIONS);
    }

    /** Measured from the center of the ice cream * */
    public static class StagingPositions {
      public static final Pose2d LEFT_ICE_CREAM =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
      public static final Pose2d MIDDLE_ICE_CREAM =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
      public static final Pose2d RIGHT_ICE_CREAM =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }
  }

  public enum Direction {
    RIGHT,
    LEFT;
  }

  public enum DetectionMode {
    CORAL,
    APRILTAG;
  }
}
