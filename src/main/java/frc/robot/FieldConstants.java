// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double fieldWidth = Units.inchesToMeters(323.277);
  public static final double wingX = Units.inchesToMeters(229.201);
  public static final double podiumX = Units.inchesToMeters(126.75);
  public static final double startingLineX = Units.inchesToMeters(74.111);

  public static final Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static final double centerlineX = fieldLength / 2.0;

    // need to update
    public static final double centerlineFirstY = Units.inchesToMeters(29.638);
    public static final double centerlineSeparationY = Units.inchesToMeters(66);
    public static final double spikeX = Units.inchesToMeters(114);
    // need
    public static final double spikeFirstY = Units.inchesToMeters(161.638);
    public static final double spikeSeparationY = Units.inchesToMeters(57);

    public static final Translation2d[] centerlineTranslations = new Translation2d[5];
    public static final Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    // corners (blue alliance origin)
    public static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    public static final Translation3d topLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d centerSpeakerOpening =
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
  }

  // currently the corner with the center face - probably needs to be the corner against the
  // alliance station wall
  public static final class Subwoofer {
    public static final Pose2d ampFaceCorner =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(239.366),
            Rotation2d.fromDegrees(-120));

    public static final Pose2d sourceFaceCorner =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(197.466),
            Rotation2d.fromDegrees(120));

    public static final Pose2d centerFace =
        new Pose2d(
            Units.inchesToMeters(35.775),
            Units.inchesToMeters(218.416),
            Rotation2d.fromDegrees(180));
  }

  public static final class Stage {
    public static final Pose2d podiumLeg =
        new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());
    public static final Pose2d ampLeg =
        new Pose2d(
            Units.inchesToMeters(220.873),
            Units.inchesToMeters(212.425),
            Rotation2d.fromDegrees(-30));
    public static final Pose2d sourceLeg =
        new Pose2d(
            Units.inchesToMeters(220.873),
            Units.inchesToMeters(110.837),
            Rotation2d.fromDegrees(30));

    public static final Pose2d centerPodiumAmpChain =
        new Pose2d(
            podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
            Rotation2d.fromDegrees(120.0));
    public static final Pose2d centerAmpSourceChain =
        new Pose2d(
            ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
    public static final Pose2d centerSourcePodiumChain =
        new Pose2d(
            sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
            Rotation2d.fromDegrees(240.0));
    public static final Pose2d center =
        new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
    public static final double centerToChainDistance =
        center.getTranslation().getDistance(centerPodiumAmpChain.getTranslation());
  }

  public static final class Amp {
    public static final Translation2d ampTapeTopCorner =
        new Translation2d(Units.inchesToMeters(130.0), Units.inchesToMeters(305.256));
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static class AutoPathPoints {
    public static final double ROBOT_Y_LENGTH = Units.inchesToMeters(34.6875);
    public static final double ROBOT_X_LENGTH = Units.inchesToMeters(34.875);

    // Robot transforms
    public static final Transform2d ROBOT_FRONT_RIGHT_CORNER_TO_CENTER =
        new Transform2d(-ROBOT_X_LENGTH / 2, ROBOT_Y_LENGTH / 2, Rotation2d.fromDegrees(0.0));
    public static final Transform2d ROBOT_FRONT_LEFT_CORNER_TO_CENTER =
        new Transform2d(-ROBOT_X_LENGTH / 2, -ROBOT_Y_LENGTH / 2, Rotation2d.fromDegrees(0.0));
    public static final Transform2d ROBOT_FRONT_TO_CENTER =
        new Transform2d(-ROBOT_X_LENGTH / 2, 0.0, Rotation2d.fromDegrees(0.0));

    // Starting poses
    public static final Pose2d SOURCE_SIDE_SUBWOOFER =
        Subwoofer.sourceFaceCorner.transformBy(ROBOT_FRONT_RIGHT_CORNER_TO_CENTER);
    public static final Pose2d AMP_SIDE_SUBWOOFER =
        Subwoofer.ampFaceCorner.transformBy(ROBOT_FRONT_LEFT_CORNER_TO_CENTER);
    public static final Pose2d CENTER_SUBWOOFER =
        Subwoofer.centerFace.transformBy(ROBOT_FRONT_TO_CENTER);
    public static final Pose2d OPPONENT_SOURCE_AGAINST_ALLIANCE_WALL =
        new Pose2d(0.39, 1.95, Rotation2d.fromDegrees(180.0));
    public static final Pose2d OPPONENT_SOURCE_AGAINST_STARTING_LINE =
        new Pose2d(1.43, 1.58, Rotation2d.fromDegrees(180.0));

    // Note poses
    public static final Pose2d NOTE_1_SHORT =
        new Pose2d(
            StagingLocations.spikeX - 1,
            StagingLocations.spikeFirstY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_2_SHORT =
        new Pose2d(
            StagingLocations.spikeX - 1,
            StagingLocations.spikeFirstY + StagingLocations.spikeSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_3_SHORT =
        new Pose2d(
            StagingLocations.spikeX - 1,
            StagingLocations.spikeFirstY + 2 * StagingLocations.spikeSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_1 =
        new Pose2d(
            StagingLocations.spikeX, StagingLocations.spikeFirstY, Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_2 =
        new Pose2d(
            StagingLocations.spikeX,
            StagingLocations.spikeFirstY + StagingLocations.spikeSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_3 =
        new Pose2d(
            StagingLocations.spikeX,
            StagingLocations.spikeFirstY + 2 * StagingLocations.spikeSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_4 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_5 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_6 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + 2 * StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_7 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + 3 * StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d NOTE_8 =
        new Pose2d(
            StagingLocations.centerlineX,
            StagingLocations.centerlineFirstY + 4 * StagingLocations.centerlineSeparationY,
            Rotation2d.fromDegrees(180.0));

    // Shot poses
    public static final Pose2d SOURCE_SIDE_SHOT =
        new Pose2d(3.20, 3.20, Rotation2d.fromDegrees(148.47));
    public static final Pose2d SOURCE_SIDE_PODIUM_SHOT =
        new Pose2d(1.9, 4.57, Rotation2d.fromDegrees(144.46));
    public static final Pose2d CENTER_SHOT = new Pose2d(2.11, 5.52, Rotation2d.fromDegrees(180));
    public static final Pose2d CENTER_SHOT_FAR = new Pose2d(3, 5.52, Rotation2d.fromDegrees(180));
    public static final Pose2d AMP_SIDE_SHOT = new Pose2d(2.10, 6.54, new Rotation2d(-153.43));

    // Misc poses
    public static final Pose2d OUT_OF_THE_WAY =
        new Pose2d(3.05, 0.39, Rotation2d.fromDegrees(180.0));
    public static final Pose2d SUBWOOFER_CENTER_DROPPED_NOTE =
        new Pose2d(1.35, 5.53, Rotation2d.fromDegrees(0.0));
  }

  public class AprilTagConstants {
    public static final List<AprilTag> APRIL_TAGS =
        List.of(
            new AprilTag(1, new Pose3d()),
            new AprilTag(2, new Pose3d()),
            new AprilTag(3, new Pose3d()),
            new AprilTag(4, new Pose3d()),
            new AprilTag(5, new Pose3d()),
            new AprilTag(6, new Pose3d()),
            new AprilTag(7, new Pose3d()),
            new AprilTag(8, new Pose3d()),
            new AprilTag(9, new Pose3d()),
            new AprilTag(10, new Pose3d()),
            new AprilTag(11, new Pose3d()),
            new AprilTag(12, new Pose3d()),
            new AprilTag(13, new Pose3d()),
            new AprilTag(14, new Pose3d()),
            new AprilTag(15, new Pose3d()),
            new AprilTag(16, new Pose3d()));
    public static final AprilTagFieldLayout FIELD_LAYOUT_2024 =
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }
}
