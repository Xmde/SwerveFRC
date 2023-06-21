package frc.robot.oi.inputs;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RisingEdgeTrigger {
    private boolean lastState = false;
    private boolean currentState = false;
    private final Trigger trigger;

    public RisingEdgeTrigger(Trigger trigger) {
        this.trigger = trigger;
    }

    public RisingEdgeTrigger(BooleanSupplier trigger) {
        this(new Trigger(trigger));
    }

    public boolean get() {
        lastState = currentState;
        currentState = trigger.getAsBoolean();
        return currentState && !lastState;
    }

    public Trigger getTrigger() {
        return trigger;
    }
}
