package frc.robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;



public final class Constants {
    public static final class VisionConstants {
        public static final double kP = 0.2; 
        public static final double kI = 0.001;
        public static final double kD = 0.005;
        public static final double kTolerance = 0.5;
    }
}
