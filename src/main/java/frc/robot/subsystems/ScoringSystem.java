// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.HWMap.Algae;
// import frc.robot.subsystems.AlgaeArmSubsystem;
// import frc.robot.subsystems.AlgaeIntakeSubsystem;
// import frc.robot.subsystems.CoralArmSubsystem;
// import frc.robot.subsystems.CoralIntakeSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import swervelib.SwerveInputStream;

// public class ScoringSystem
// {

//   private SwerveSubsystem      m_swerve;
//   private SwerveInputStream    m_swerveInputStream;
//   private TargetingSystem      m_targetSystem;

//   public ScoringSystem(SwerveSubsystem swerve, SwerveInputStream driveStream)
//   {
//     m_swerveInputStream = driveStream;
//   }

//   ///  Score Coral Command for PathPlanner that does not move the swerve drive at all keeping the
// pathplanner auto
//   /// intact.

//   public Command scoreCoral()
//   {
//     // Arm down, elevator down, drive backwards x in
//     return Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
//                       m_algaeArm.setAlgaeArmAngle(-40).andThen(m_algaeArm.hold().repeatedly()),
//
// m_coralArm.getCoralCommand(m_targetSystem).andThen(m_coralArm.hold(false).repeatedly()),
//                              Commands.waitSeconds(0.3).andThen(m_coralIntake.wristScore()))
//
// .until(m_elevator.atCoralHeight(m_targetSystem).and(m_coralArm.atCoralAngle(m_targetSystem))
//                                     .and(m_coralIntake.atScoringAngle())).withTimeout(2)
//                    .andThen(Commands.parallel(
//                                         m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
//                                         m_coralArm.hold(false).repeatedly(),
//
// m_algaeArm.setAlgaeArmAngle(-40).andThen(m_algaeArm.hold().repeatedly()),
//
// Commands.waitSeconds(0.3).andThen(m_coralIntake.wristScore()))
//                                     .withDeadline(m_targetSystem.driveToCoralTarget(m_swerve)))
//
// .andThen(m_coralIntake.wristScore().withDeadline(m_coralArm.score().withTimeout(1)))
//
// .andThen(m_swerve.driveBackwards().alongWith(m_coralIntake.wristIntake(),m_coralArm.hold(false).repeatedly()).withTimeout(0.5))
//                                     .andThen(restArmsSafe());
//   }

//   ///  Autonomous command for scoring the algae arm
//   public Command scoreAlgaeProcessorAuto()
//   {
//     //set elevator height, set algae angle, spit out ball, drive pose
//     return Commands.parallel(m_elevator.AlgaePROCESSOR().repeatedly(),
// m_algaeArm.PROCESSOR().repeatedly()).until(
//
// m_elevator.aroundAlgaePROCESSOR().and(m_algaeArm.aroundPROCESSORAngle())).withTimeout(5)
//                    .andThen(Commands.parallel(m_algaeIntake.out(),
//                                               m_elevator.AlgaePROCESSOR().repeatedly(),
//                                               m_algaeArm.PROCESSOR().andThen(m_algaeArm.hold()))
//                                     .until(() -> m_algaeArm.algaeScored())
//                                     .withTimeout(1));
//   }

//   public Command scoreAlgaeProcessor()
//   {

//     //set elevator height, set algae angle, spit out ball, drive pose
//     return m_swerve.driveToProcessor()
//                    .andThen(Commands.parallel(m_elevator.AlgaePROCESSOR().repeatedly(),
//                                               m_algaeArm.PROCESSOR().repeatedly(),
//                                               m_swerve.lockPos())
//                                     .until(m_elevator.aroundAlgaePROCESSOR()
//                                                      .and(m_algaeArm.aroundPROCESSORAngle()))
//                                     .withTimeout(5))
//                    .andThen(Commands.parallel(m_algaeIntake.out(),
//                                               m_elevator.AlgaePROCESSOR()
//                                                         .repeatedly(),
//                                               m_algaeArm.PROCESSOR().andThen(m_algaeArm.hold()),
//                                               m_swerve.lockPos())
//                                     .until(() -> m_algaeArm.algaeScored()).withTimeout(1));

//   }

//   public Command scoreAlgaeNet()
//   {
//     //set elevator height, set alage angle, spit out ball, drive pose
//     return Commands.parallel(m_algaeArm.NET().andThen(m_algaeArm.hold()),
// m_elevator.AlgaeNET().repeatedly())
//
// .withTimeout(5).until(m_elevator.aroundAlgaeNET().and(m_algaeArm.aroundNETAngle()))
//                    .andThen(Commands.parallel(m_algaeArm.NET().andThen(m_algaeArm.hold()),
//                                               m_elevator.AlgaeNET().repeatedly(),
//                                               m_algaeIntake.out())
//                                     .withTimeout(2)
//                                     .until(() -> m_algaeArm.algaeScored()));
//   }

// }

// //                     WALDO
// ///                     ( ) /-----\
// ///                  |||||||||
// //                   ((o)-(o))
// //                    |  W  |
// //                    --| |--
