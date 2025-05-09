package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Setpoints {
  public static class AutoScoring {
    public static class Processor {
      public static final Transform2d offset =
          new Transform2d(
              Inches.of(24).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));
    }

    public static class Reef {
      // x + front ->, y + left
      public static final Transform2d coralOffset =
          new Transform2d(.6, 0, Rotation2d.fromDegrees(180));
      public static final Transform2d coralShiftOffset =
          new Transform2d(-.7, 0, Rotation2d.fromDegrees(0));
      public static final Transform2d algaeOffset =
          new Transform2d(
              Inches.of(24).in(Meters), Inches.of(-19).in(Meters), Rotation2d.fromDegrees(180));
    }

    public static class HumanPlayer {

      public static class Left {

        public static final Transform2d offset =
            new Transform2d(
                Inches.of(24).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));
      }

      public static class Right {

        public static final Transform2d offset =
            new Transform2d(
                Inches.of(24).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));
      }
    }
  }
}
