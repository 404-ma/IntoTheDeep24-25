package org.firstinspires.ftc.teamcode.Helper.ReactiveState;

import java.lang.reflect.Field;
import java.util.Arrays;

public class Reactive {
    public static void init(Object obj) {
        Class<?> clazz = obj.getClass();

        for (Field field : clazz.getDeclaredFields()) {
            if (field.isAnnotationPresent(StateChange.class)) {
                StateChange stateChange = field.getAnnotation(StateChange.class);

                assert stateChange != null;

                if (ReactiveState.class.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    try {
                        ReactiveState reactiveState = (ReactiveState) field.get(obj);
                        assert reactiveState != null;
                        reactiveState.setOnChangeMethodName(stateChange.value());
                        reactiveState.setClass(obj);

                    } catch (Exception e) {
//                        System.out.println(e.toString());
                    }
                }
            }
        }
    }
}