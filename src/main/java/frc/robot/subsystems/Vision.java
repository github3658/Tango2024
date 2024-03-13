// This is the vision subsystem. 
// This essentially wraps calls to Limelight Lib, NetworkTables, and more

package frc.robot.subsystems;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    /**
     * An enum of GENERIC AprilTag targets we could align to. These will be converted to Blue Alliance and Red Alliance-specific AprilTags internally. Includes:
     * <p> Amp
     * <p> SpeakerCenter
     * <p> SpeakerSouth
     * 
     */
    enum AprilTags {
        Amp,
        SpeakerCenter,
        SpeakerSouth;
    }
    
    /**
     * An enum of the BLUE ALLIANCE AprilTag targets we could align to. Includes:
     * <p> Amp (6)
     * <p> SpeakerCenter (7)
     * <p> SpeakerSouth (8)
     * 
     */
    enum AprilTagsBlueAlliance {
        Amp(6),
        SpeakerCenter(7),
        SpeakerSouth(8);
        public final int value;
        AprilTagsBlueAlliance(int value) {
            this.value = value;
        }
    }

    /**
     * An enum of the RED ALLIANCE AprilTag targets we could align to. Includes:
     * <p> SpeakerSouth (3)
     * <p> SpeakerCenter (4)
     * <p> Amp (5)
     * 
     */
    enum AprilTagsRedAlliance {
        SpeakerSouth(3),
        SpeakerCenter(4),
        Amp(5);
        public final int value;
        AprilTagsRedAlliance(int value) {
            this.value = value;
        }
    }

    /* CONSTANTS (prefix: c) */

    /* OTHER VARIABLES */
    PoseEstimator m_PoseEstimator;

    private LED s_LED;
    public Vision(LED led) {
        s_LED = led;
        m_PoseEstimator = new PoseEstimator<>(null, null, null, null);
    }

    @Override
    public void periodic() {
    }

    public void stop() {
    }

    public void AlignToApril() {
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
    }
}