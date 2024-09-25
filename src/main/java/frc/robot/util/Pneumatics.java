package frc.robot.util;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {
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

    public Solenoid createSolenoid(int channel) {return pcm.makeSolenoid(channel);}
}

