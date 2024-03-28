package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.*;


import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class Testing {

    private final TalonFX leadFalcon = new TalonFX(0);

    

    public void setVoltage() {
        leadFalcon.setVoltage(3);
    }
    
}
