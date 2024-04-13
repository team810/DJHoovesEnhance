package frc.robot.util;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private static Pneumatics INSTANCE;
    public static Pneumatics getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Pneumatics();
        }
        return INSTANCE;
    }

    private final PneumaticsControlModule pcm = new PneumaticsControlModule(20);




    private Pneumatics() {
        pcm.enableCompressorDigital();
    }


    public DoubleSolenoid createSolenoid(int fwd, int back)
    {
        return pcm.makeDoubleSolenoid(fwd,back);
    }


    @Override
    public void periodic() {


    }
}

