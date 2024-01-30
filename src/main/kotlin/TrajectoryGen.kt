import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 18.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPose = Pose2d(36.0, -63.0, 90.0.toRadians)

    private var lastPose = startPose;

    private var spikePose = startPose;

    fun createTrajectory(): ArrayList<Trajectory> {

        val angle = -20.0


        if (angle < -15.0) {
            spikePose = Pose2d(-9.0, 18.0, 90.0.toRadians)
        } else if (angle < 15.0) {
            spikePose = Pose2d(0.0, 33.0, 0.0.toRadians)
        } else {
            spikePose = Pose2d(12.0, 33.0, 0.0.toRadians)
        }

        val list = ArrayList<Trajectory>()

        val traj1 = TrajectoryBuilder(lastPose, lastPose.heading, combinedConstraints)
            .splineTo(lastPose.vec() + spikePose.vec(), lastPose.heading + spikePose.heading)
            .build()

        lastPose = traj1.end()

        val traj2 = TrajectoryBuilder(lastPose, true, combinedConstraints)
            .splineTo(lastPose.vec() - spikePose.vec(), lastPose.heading-180.0.toRadians)
            .build()

        lastPose = traj2.end()

//        val traj2 = TrajectoryBuilder(traj1.end(), traj1.end().heading, combinedConstraints)
//            .splineTo(traj1.end().vec() + Vector2d(15.0, 15.0), traj1.end().heading - 90.0.toRadians)
//            .build()
//        val traj2 = TrajectoryBuilder(lastPose, lastPose.heading, combinedConstraints)
//            .lineTo()

        list.add(traj1)
        list.add(traj2)

        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
