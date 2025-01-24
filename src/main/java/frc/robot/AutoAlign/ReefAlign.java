package frc.robot.AutoAlign;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefAlign {

    //Blue Alliance Reef

        //Facing the Lower Blue Coral Station
    Pose2d ID_17_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_17_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_17_ANGLE);
    Pose2d ID_17_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_17_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_17_ANGLE);

       //Facing the Drive Stations
    Pose2d ID_18_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_18_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_18_ANGLE);
    Pose2d ID_18_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_18_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_18_ANGLE);

    //Facing the Upper Blue Coral Station
    Pose2d ID_19_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_19_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_19_ANGLE);
    Pose2d ID_19_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_19_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_19_ANGLE);
    
    //Facing the Blue Barge
    Pose2d ID_20_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_20_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_20_ANGLE);
    Pose2d ID_20_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_20_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_20_ANGLE);
    
   //Facing the Middle of the barge
    Pose2d ID_21_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_21_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_21_ANGLE);
    Pose2d ID_21_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_21_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_21_ANGLE); 

    //Facing the Red Barge
    Pose2d ID_22_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_22_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_22_ANGLE);
    Pose2d ID_22_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_22_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_22_ANGLE);
    


    //RED Alliance Reef
    Pose2d ID_6_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_6_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_6_ANGLE);
    Pose2d ID_6_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_6_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_6_ANGLE);

       //Facing the Drive Stations
    Pose2d ID_7_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_7_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_7_ANGLE);
    Pose2d ID_7_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_7_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_7_ANGLE);

    //Facing the Upper Blue Coral Station
    Pose2d ID_8_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_8_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_8_ANGLE);
    Pose2d ID_8_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_8_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_8_ANGLE);
    
    //Facing the Blue Barge
    Pose2d ID_9_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_9_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_9_ANGLE);
    Pose2d ID_9_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_9_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_9_ANGLE);
    
   //Facing the Middle of the barge
    Pose2d ID_10_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_10_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_10_ANGLE);
    Pose2d ID_10_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_10_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_10_ANGLE); 

    //Facing the Red Barge
    Pose2d ID_11_LEFT = new Pose2d(Constants.ReefAlignConstants.ID_11_LEFT_TRANSLATION,Constants.ReefAlignConstants.ID_11_ANGLE);
    Pose2d ID_11_RIGHT = new Pose2d(Constants.ReefAlignConstants.ID_11_RIGHT_TRANSLATION,Constants.ReefAlignConstants.ID_11_ANGLE);
}