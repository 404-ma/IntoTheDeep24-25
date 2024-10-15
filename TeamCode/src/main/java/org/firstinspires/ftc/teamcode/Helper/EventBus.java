package org.firstinspires.ftc.teamcode.Helper;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;

/*
example use:

use EventBus.getInstance() to retrieve the singleton instance of the EventBus

// you can edit return of onMessage when instantiating
abstract class TestSub implements Subscriber<Void> {
    @Override
    public Void onMessage() {
        // add ur onMessage code here

        return null;
    }
}

// for parameters in the Subscriber extension:
abstract class SubWithParams implements Subscriber<String> {
    public String onMessage(String name) {
        return "Hello"+name;
    }
}

// add a subscription of id 'telemetry'
EventBus.getInstance().subscribe("telemetry", TestSub.class);

// emitting a new event
Set<String> targets = new HashSet<>();
targets.add("telemetry"); // add a new string to `targets` for each subId you want to activate
EventBus.getInstance().emit(targets);

 */

public class EventBus {
    private static final EventBus instance = new EventBus();

    private final Map<String, Class<? extends Subscriber<?>>> subscribers = new HashMap<>();

    public EventBus() {

    }

    public static EventBus getInstance() {
        return instance;
    }

    public void subscribe(String subId, Class<? extends Subscriber<?>> sub) {
        subscribers.computeIfAbsent(subId, k -> sub);
    }

    public void unsubscribe(String subId) {
        subscribers.remove(subId);
    }

    public void emit(Set<String> events, Object ... params) {
        for (Map.Entry<String, Class<? extends Subscriber<?>>> entry : this.subscribers.entrySet()) {
            String subId = entry.getKey();
            Class<? extends Subscriber<?>> subscriberClass = entry.getValue();

            if (events.contains(subId)) {
                try {
                    Subscriber<?> subscriberInstance = subscriberClass.getDeclaredConstructor().newInstance();

                    subscriberInstance.onMessage(params);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
