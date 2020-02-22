package frc.robot.utils;

import java.util.function.BooleanSupplier;

public class ToggledBoolean {

    private boolean lastState = true;

    private boolean toggled = false;
    private BooleanSupplier state;

    public ToggledBoolean(BooleanSupplier booleanSupplier) {
        this.state = booleanSupplier;
    }

    public boolean getToggled() {
        return toggled;
    }

    public boolean getState(){
        return state.getAsBoolean();
    }

    public void update() {
        toggled = state.getAsBoolean() && !lastState;

        lastState = state.getAsBoolean();
    }

}
