/*
WARNING: DEPENDENCY INJECTOR IS ONLY TESTED ON SINGLE THREAD. DO NOT USE FOR MULTI-THREADED STUFF (before testing it)
 */

package org.firstinspires.ftc.teamcode.Helper.DependencyInjection;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;


public class DependencyInjector {
    private static final Map<String, Object> cache = new ConcurrentHashMap<>();

    public static void register(String name, Object instance) {
        cache.put(name, instance);
    }

    public static void unregister(String name) {
        cache.remove(name);
    }

    public static Object resolve(String name) throws Exception {
        if (!cache.containsKey(name)) {
            throw new Exception("No cached dependency found for: " + name);
        }
        return cache.get(name);
    }

    public static void inject(Object obj) throws Exception {
        Class<?> clazz = obj.getClass();

        for (Field field : clazz.getDeclaredFields()) {
            if (field.isAnnotationPresent(Inject.class)) {
                Inject inject = field.getAnnotation(Inject.class);

                assert inject != null;
                String cacheName = inject.value();

                Object dependency = resolve(cacheName);

                if (!field.getType().isInstance(dependency)) {
                    throw new Exception("Type mismatch: Cannot assign " + dependency.getClass().getName() +
                            " to field " + field.getName() + " of type " + field.getType().getName()+": "+"Occurred in Injectable \""+cacheName+"\"");
                }

                field.setAccessible(true);

                field.set(obj, dependency);
            }
        }
    }
}