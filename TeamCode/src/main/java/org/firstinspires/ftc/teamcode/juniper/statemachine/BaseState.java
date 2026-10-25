package org.firstinspires.ftc.teamcode.juniper.statemachine;

import com.example.juniper.util.BetterTimer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class BaseState implements State {

    private List<Transition> transitions;
    private boolean running = false;
    private final String name;
    private Runnable entry, exit, during;
    private long minTime = 0, maxTime = Long.MAX_VALUE;
    private State nextState;
    private final BetterTimer timer = new BetterTimer();

    public BaseState(String name) {
        this.name = name;
        this.transitions = new ArrayList<>();
    }

    public void start() {
        timer.resetTimer();
        running = true;
        if (entry != null) {
            entry.run();
        }
        for (Transition t : transitions) {
            t.start();
        }
    }

    public void run() {
        if (!running) return;

        if (during != null) during.run();

        long elapsedNs = timer.getElapsedTimeNs();

        if (elapsedNs > minTime) {
            for (Transition t : transitions) {
                t.run();
                if (t.isFinished()) {
                    State transitionTarget = t.getNextState();
                    if (exit != null) exit.run();
                    nextState = transitionTarget;
                    running = false;
                    return;
                }
            }
        }

        if (elapsedNs >= maxTime) {
            if (exit != null) exit.run();
            running = false;
            //goes to fallback state
        }
    }

    public BaseState setEntry(Runnable function) {
        this.entry = function;
        return this;
    }
    public BaseState setExit(Runnable function) {
        this.exit = function;
        return this;
    }
    public BaseState setDuring(Runnable function) {
        this.during = function;
        return this;
    }

    public BaseState addTransition(Transition transition) {
        transitions.add(transition);
        return this;
    }

    public boolean transitionsAreTrue() {
        if (timer.getElapsedTimeNs() > minTime) {
            for (Transition t : transitions) {
                t.run();
                if (t.isFinished()) {
                    return true;
                }
            }
        }

        return false;
    }

    public BaseState setMinTimeSeconds(double seconds) {
        this.minTime = (long) (seconds * 1_000_000_000L);
        return this;
    }
    public BaseState setMinTimeMS(long millis) {
        this.minTime = millis * 1_000_000L;
        return this;
    }
    public BaseState setMinTimeNS(long nanos) {
        this.minTime = nanos;
        return this;
    }

    public BaseState setMaxTimeSeconds(double seconds) {
        this.maxTime = (long) (seconds * 1_000_000_000L);
        return this;
    }
    public BaseState setMaxTimeMS(long millis) {
        this.maxTime = millis * 1_000_000L;
        return this;
    }
    public BaseState setMaxTimeNS(long nanos) {
        this.maxTime = nanos;
        return this;
    }

    public BaseState setFallbackState(State fallbackState) {
        if (fallbackState == null) {
            throw new IllegalArgumentException("Fallback State cannot be null");
        }
        this.nextState = fallbackState;
        return this;
    }

    public boolean isFinished() {
        return !running;
    }

    public String getName() {
        return name;
    }

    @Override
    public State getNextState() {
        return nextState;
    }

    public void addTransition(BooleanSupplier booleanSupplier, String done) {
    }
}