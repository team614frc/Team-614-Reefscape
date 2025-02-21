package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
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
    public static final Pose3d CAMERA_OFFSET = Pose3d.kZero;
    public static final double ADJUST_X = Units.inchesToMeters(0);
    public static final double ADJUST_Y = Units.inchesToMeters(0);
  }

  public static class Reef {
    public static final Translation2d CENTER =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

    /** Side of the reef to the inside of the reef zone line * */
    public static final double FACE_TO_ZONE_LINE = Units.inchesToMeters(12);

    /** Starting facing the driver station in clockwise order * */
    public static final List<Pose2d> CENTER_FACES_BLUE =
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

    public static final List<Pose2d> CENTER_FACES_RED = new ArrayList<>(5);

    static {
      for (int i = 0; i < CENTER_FACES_RED.size(); i++) {
        CENTER_FACES_RED.add(CENTER_FACES_BLUE.get(i));
      }
    }

    public static final List<Integer> CENTER_FACES_RED_IDS = List.of(7, 6, 11, 10, 9, 8);

    public static final List<Integer> CENTER_FACES_BLUE_IDS = List.of(18, 17, 22, 21, 20, 19);

    /** Starting at the right branch facing the driver station in clockwise * */
    public static final List<Map<ReefLevel, Pose3d>> BRANCH_POSITIONS = new ArrayList<>(13);

    static {
      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(CENTER, Rotation2d.fromDegrees(180 - (60 * face)));

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(
                              new Transform2d(Offsets.ADJUST_X, Offsets.ADJUST_Y, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(
                              new Transform2d(Offsets.ADJUST_X, Offsets.ADJUST_Y, new Rotation2d()))
                          .getY(),
                      level.height.in(Meters)),
                  new Rotation3d(
                      0, level.pitch.in(Radians), poseDirection.getRotation().getRadians())));

          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  Offsets.ADJUST_X, -Offsets.ADJUST_Y, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  Offsets.ADJUST_X, -Offsets.ADJUST_Y, new Rotation2d()))
                          .getY(),
                      level.height.in(Meters)),
                  new Rotation3d(
                      0, level.pitch.in(Radians), poseDirection.getRotation().getRadians())));
        }
        BRANCH_POSITIONS.add(fillLeft);
        BRANCH_POSITIONS.add(fillRight);
      }
    }

    public static final Map<Integer, Map<Direction, Integer>> POSITION_MAP =
        Map.ofEntries(
            Map.entry(7, Map.of(Direction.RIGHT, 0, Direction.LEFT, 1)),
            Map.entry(18, Map.of(Direction.RIGHT, 0, Direction.LEFT, 1)),
            Map.entry(6, Map.of(Direction.RIGHT, 2, Direction.LEFT, 3)),
            Map.entry(17, Map.of(Direction.RIGHT, 2, Direction.LEFT, 3)),
            Map.entry(11, Map.of(Direction.RIGHT, 4, Direction.LEFT, 5)),
            Map.entry(22, Map.of(Direction.RIGHT, 4, Direction.LEFT, 5)),
            Map.entry(10, Map.of(Direction.RIGHT, 6, Direction.LEFT, 7)),
            Map.entry(21, Map.of(Direction.RIGHT, 6, Direction.LEFT, 7)),
            Map.entry(9, Map.of(Direction.RIGHT, 8, Direction.LEFT, 9)),
            Map.entry(20, Map.of(Direction.RIGHT, 8, Direction.LEFT, 9)),
            Map.entry(8, Map.of(Direction.RIGHT, 10, Direction.LEFT, 11)),
            Map.entry(19, Map.of(Direction.RIGHT, 10, Direction.LEFT, 11)));
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

  public enum Direction {
    RIGHT,
    LEFT;
  }

  public enum DetectionMode {
    CORAL,
    APRILTAG;
  }

  public enum ReefLevel {
    L4(Inches.of(72), Degrees.of(-90)),
    L3(Inches.of(47.625), Degrees.of(-35)),
    L2(Inches.of(31.875), Degrees.of(-35)),
    L1(Inches.of(18), Degrees.of(0));

    ReefLevel(Distance height, Angle pitch) {
      this.height = height;
      this.pitch = pitch;
    }

    public final Distance height;
    public final Angle pitch;
  }
}
