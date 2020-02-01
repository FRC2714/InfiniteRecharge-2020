package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;

public class ToggledBreakBeam {
    private boolean lastState = true;
    private DigitalInput beam;

    private boolean toggled = false;

    public ToggledBreakBeam(DigitalInput beam) {
        this.beam = beam;
    }

    public boolean getToggled() {
        return toggled;
    }

    public boolean getState(){
        return beam.get();
    }

    public void update() {
        if (beam.get() && !lastState)
            toggled = true;
        else
            toggled = false;

        lastState = beam.get();
    }
}
