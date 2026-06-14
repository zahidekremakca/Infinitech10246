package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable m_table;
    
    public VisionSubsystem() {
        m_table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        // Debug için verileri ekrana basalım
        SmartDashboard.putNumber("Vision/TargetX", getTargetYaw());
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/TargetID", getTargetID());
    }

    public double getTargetYaw() {
        // Limelight'tan gerçek tx değerini çek
        return m_table.getEntry("tx").getDouble(0.0);
    }

    public boolean hasTarget() {
        // tv (target valid) 1.0 ise hedef vardır
        return m_table.getEntry("tv").getDouble(0.0) > 0.5;
    }

    public double getTargetID() {
        // tid (Target ID) değerini çek, hedef yoksa -1 döndür
        return m_table.getEntry("tid").getDouble(-1.0);
    }
}