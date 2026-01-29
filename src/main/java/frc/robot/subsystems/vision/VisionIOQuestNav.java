package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;

/** IO implementation for QuestNav */
public class VisionIOQuestNav implements VisionIO {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;
  private final Alert lowBatteryAlert =
      new Alert("QuestNav battery below 50%! Charge it!!!", AlertType.kWarning);

  public VisionIOQuestNav() {
    this.questNav = new QuestNav();
    this.robotToQuest = VisionConstants.robotToQuestTransform;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    questNav.commandPeriodic();

    inputs.connected = questNav.isConnected();
    questNav.getBatteryPercent().ifPresent(battery -> lowBatteryAlert.set(battery < 50.0));

    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    List<PoseObservation> observations = new LinkedList<>();

    for (PoseFrame frame : frames) {
      if (!frame.isTracking()) continue;

      Pose3d questPose = frame.questPose3d();
      Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

      observations.add(
          new PoseObservation(
              frame.dataTimestamp(),
              robotPose,
              0.0, // no ambiguity concept
              2, // placeholder tagCount
              0.0, // no distance metric
              PoseObservationType.QUESTNAV));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagIds = new int[0];
  }

  /** reset QuestNav pose */
  public void setPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(robotToQuest);
    questNav.setPose(questPose);
  }
}
