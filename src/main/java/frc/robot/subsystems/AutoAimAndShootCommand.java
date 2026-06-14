//ROBOTUN KONUMUNA GÖRE OTOMATIK MOTOR GÜCÜ VE HİZALAMA KOMUTU
package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimAndShootCommand extends Command{
    
    private final ShooterSubsystem shooter;

    // --- 2. İNTERPOLASYON HARİTASI (Mesafe -> RPM) ---
    private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    
    
    
    
    private final double LENS_HEIGHT = 0.5; // Kameranın yerden yüksekliği
    private final double TARGET_HEIGHT = 1.2573; // Hub'ın yerden yüksekliği
    private final double CAMERA_MOUNT_ANGLE = 30.0; // Kameranın yukarı doğru bakış açısı (Derece)

    public AutoAimAndShootCommand(CommandSwerveDrivetrain swerve, ShooterSubsystem shooter) {
        // Not: 'swerve' parametresini constructor'da bırakıyoruz ki diğer dosyalar hata vermesin 
        // ama 'addRequirements' kısmına eklemiyoruz!
        this.shooter = shooter;
        addRequirements(shooter); 
        
        // Atölyede yaptığınız deneme atışlarının sonuçlarını buraya giriyoruz.
        // Sol taraf: Limelight'ın ölçtüğü mesafe (Metre) | Sağ taraf: Motor RPM'i
        shooterMap.put(1.0, 3500.0); 
        shooterMap.put(1.5, 3700.0);
        shooterMap.put(2.0, 3900.0);
        shooterMap.put(2.5, 4100.0);
        shooterMap.put(3.0, 4300.0);
        shooterMap.put(3.5, 4500.0);
        shooterMap.put(4.0, 4700.0);
        
        // tx bir offset değeridir, dairesel hesaplama (continuous input) rotasyonu bozar, bu yüzden kapalı kalmalı.
    }

    @Override
    public void execute() {
        // --- LİMELİGHT VERİLERİNİ OKUMA ---
        var limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = limelightTable.getEntry("tv").getDouble(0);
        
        double ty = limelightTable.getEntry("ty").getDouble(0);
        double tid = limelightTable.getEntry("tid").getDouble(-1); // AprilTag ID'sini oku

        // Sadece 9, 10, 25 ve 26 numaralı ID'leri kabul et
        boolean isCorrectTag = (tid == 9 || tid == 10 || tid == 25 || tid == 26);
        
        // --- A. EĞER HEDEFİ GÖRÜYORSAK VE İSTEDİĞİMİZ TAG İSE ---
        if (tv == 1.0 && isCorrectTag) {
            
            // 1. Mesafe Hesaplama (Trigonometri)
            double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
            
            // Mesafeyi hesapla ve 1.0m ile 4.0m arasında sınırla
            double currentDistance = (TARGET_HEIGHT - LENS_HEIGHT) / Math.tan(Math.max(angleToGoalRad, 0.01));
            currentDistance = MathUtil.clamp(currentDistance, 1.0, 4.0);
            
            // 2. Mesafeye Göre Motorlara Güç Verme (İnterpolasyon)
            double targetRPM = shooterMap.get(currentDistance);
            shooter.runShooterWithRPM(targetRPM); // Alt sistemdeki metodla motorları sür

            // Takip için verileri Dashboard'a gönder
            SmartDashboard.putNumber("Shooter/Calculated Distance", currentDistance);
            SmartDashboard.putNumber("Shooter/AutoAim Target RPM", targetRPM);
            
        } 
        // --- B. EĞER HEDEF GÖRÜNMÜYORSA (tv == 0) SABİT HIZDA ATİŞ ---
        else {
            // Hedef kaybolduğunda motorları durdurmak yerine sabit bir RPM ile çalıştır
            shooter.runShooterWithRPM(4000.0); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Tuş bırakıldığında veya komut kesildiğinde her şeyi güvenlice durdur
        shooter.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Bu komut, tuşa (Örn: R1) basılı tutulduğu sürece çalışacağı için hiçbir zaman kendi kendine bitmez.
        return false; 
    }
}

//ROBOTUN KONUMU SABİT OLAN VE HİZALAMA YAPAN KOMUT
/*package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest; // CTRE Sürüş İsteği

import frc.robot.subsystems.CommandSwerveDrivetrain; // Sizin Swerve dosyanızın adı
import frc.robot.subsystems.ShooterSubsystem; // Sizin Atıcı dosyanızın adı (Eğer varsa)

public class AutoAimAndShootCommand extends Command {
    
    // --- KULLANACAĞIMIZ ALT SİSTEMLER ---
    private final CommandSwerveDrivetrain swerve;
    private final ShooterSubsystem shooter;
    
    // --- 1. CTRE SÜRÜŞ İSTEĞİ (Robot Merkezli) ---
    // Robotun kendi etrafında dönüp, kendi önü yönünde ileri/geri gitmesi için
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // --- 2. PID KONTROLCÜLERİ ---
    // Dönüş (Sağ/Sol) PID'si -> Hedefi ortalamak için
    private final PIDController turnPID = new PIDController(0.05, 0.0, 0.005);
    
    // Sürüş (İleri/Geri) PID'si -> Mesafeyi ayarlamak için
    // P değerini küçük başlatın (0.1 gibi), robot çok yavaş geliyorsa yavaşça artırın.
    private final PIDController drivePID = new PIDController(0.1, 0.0, 0.0);
    
    // --- 3. SABİTLER (Sizin ölçeceğiniz değerler) ---
    private final double TARGET_DISTANCE = 1.0; // Robotun gitmesini istediğiniz sabit atış mesafesi (Metre)
    private final double HUB_X = 16.0; // Kör nokta dönüşü için saha koordinatı
    private final double HUB_Y = 5.5;  // Kör nokta dönüşü için saha koordinatı
    
    private final double LENS_HEIGHT = 0.5; // Kamera yerden yüksekliği (Metre)
    private final double TARGET_HEIGHT = 2.64; // Hub yüksekliği
    private final double CAMERA_MOUNT_ANGLE = 30.0; // Kamera açısı

    public AutoAimAndShootCommand(CommandSwerveDrivetrain swerve, ShooterSubsystem shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
        addRequirements(swerve, shooter); 
        
        turnPID.enableContinuousInput(-Math.PI, Math.PI); 
    }

    @Override
    public void execute() {
        // --- LİMELİGHT VERİLERİNİ OKUMA ---
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        
        double omega = 0; // Dönüş hızımız (rad/s)
        double vx = 0;    // İleri/Geri hızımız (m/s)
        
        if (tv == 1.0) {
            // --- DURUM A: HEDEF GÖRÜNÜYOR (Hem hizalan hem mesafeye git) ---
            
            // 1. Dönüş Hizalaması
            omega = turnPID.calculate(tx, 0); 
            
            // 2. Mesafe Hesaplaması
            double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
            double currentDistance = (TARGET_HEIGHT - LENS_HEIGHT) / Math.tan(angleToGoalRad);
            
            // 3. Mesafe PID'si ile İleri/Geri Hız Üretme
            // Mevcut mesafeden, hedef mesafeye (3.0m) gitmeye çalışır.
            vx = drivePID.calculate(currentDistance, TARGET_DISTANCE);
             
            // Atıcıyı aç (mevcut ShooterSubsystem içinde tanımlı olan yöntem)
            shooter.shootSequenceCommand(); 
            
        } else {
            // --- DURUM B: HEDEF GÖRÜNMÜYOR (Kör Nokta Dönüşü) ---
            // Hedef görünmüyorsa ileri/geri GİTME! Sadece dönerek hedefi ara.
            vx = 0; 
            
            
            Pose2d currentPose = swerve.samplePoseAt(edu.wpi.first.wpilibj.Timer.getFPGATimestamp()).orElse(new Pose2d()); 
            
            double deltaX = HUB_X - currentPose.getX();
            double deltaY = HUB_Y - currentPose.getY();
            
            double targetAngleRad = Math.atan2(deltaY, deltaX);
            double currentAngleRad = currentPose.getRotation().getRadians();
            
            omega = turnPID.calculate(currentAngleRad, targetAngleRad);
        }

        // --- GÜVENLİK VE SINIRLAMALAR (En kritik kısım) ---
        // Robotun maksimum dönüş hızı 4 rad/s
        omega = Math.max(-4.0, Math.min(omega, 4.0));
        
        // Robotun ileri/geri maksimum hızı 2 m/s (Güvenlik için çok yüksek tutmayın)
        vx = Math.max(-2.0, Math.min(vx, 2.0));
        
        // --- CTRE ŞASEYE KOMUTU GÖNDER ---
        // Artık hem vx (ileri/geri) hem de omega (dönüş) değerimiz var! Y (Sağ/Sol) hızı 0.
        swerve.setControl(driveRequest.withVelocityX(vx).withVelocityY(0).withRotationalRate(omega));
    }

    @Override
    public void end(boolean interrupted) {
        // Tuş bırakıldığında robotu çivi gibi olduğu yere çak ve durdur.
        swerve.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        shooter.shooterKapatKomutu();
    }
    
    @Override
    public boolean isFinished() {
        return false; 
    }
}*/