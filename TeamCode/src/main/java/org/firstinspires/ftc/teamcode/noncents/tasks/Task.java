package org.firstinspires.ftc.teamcode.noncents.tasks;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public interface Task {

    void init();

    boolean update();

    boolean isCancellable();

    Set<Object> getResources();

    static Task empty() {
        return new Task() {
            @Override
            public void init() {}

            @Override
            public boolean update() {
                return true;
            }

            @Override
            public boolean isCancellable() {
                return false;
            }

            @Override
            public Set<Object> getResources() {
                return Collections.emptySet();
            }
        };
    }

    static Task newWithUpdate(BooleanSupplier update) {
        return Task.empty().withUpdate(update);
    }

    static Task newWithContinuous(Runnable runnable) {
        return Task.empty().withContinuous(runnable);
    }

    static Task newWithOneshot(Runnable runnable) {
        return Task.empty().withOneshot(runnable);
    }

    static Task defer(Supplier<Task> supplier) {
        return new Task() {
            Task task;

            @Override
            public void init() {
            }

            boolean first = true;

            @Override
            public boolean update() {
                if (first) {
                    task = supplier.get();
                    task.init();
                    first = false;
                }
                return task.update();
            }

            @Override
            public boolean isCancellable() {
                return false;
            }

            @Override
            public Set<Object> getResources() {
                // return task.getResources();
                return Collections.emptySet();
            }
        };
    }

    default Task withInit(Runnable init) {
        return new Task() {
            @Override
            public void init() {
                init.run();
            }

            @Override
            public boolean update() {
                return Task.this.update();
            }

            @Override
            public boolean isCancellable() {
                return Task.this.isCancellable();
            }

            @Override
            public Set<Object> getResources() {
                return Task.this.getResources();
            }
        };
    }

    default Task withUpdate(BooleanSupplier update) {
        return new Task() {
            @Override
            public void init() {
                Task.this.init();
            }

            @Override
            public boolean update() {
                return update.getAsBoolean();
            }

            @Override
            public boolean isCancellable() {
                return Task.this.isCancellable();
            }

            @Override
            public Set<Object> getResources() {
                return Task.this.getResources();
            }
        };
    }

    default Task withCancellable(boolean cancellable) {
        return new Task() {
            @Override
            public void init() {
                Task.this.init();
            }

            @Override
            public boolean update() {
                return Task.this.update();
            }

            @Override
            public boolean isCancellable() {
                return cancellable;
            }

            @Override
            public Set<Object> getResources() {
                return Task.this.getResources();
            }
        };
    }

    default Task withResources(Object... resources) {
        return this.withResources(Set.of(resources));
    }

    default Task withResources(Set<Object> resources) {
        Set<Object> unmodifiableResources = Collections.unmodifiableSet(resources);
        return new Task() {
            @Override
            public void init() {
                Task.this.init();
            }

            @Override
            public boolean update() {
                return Task.this.update();
            }

            @Override
            public boolean isCancellable() {
                return Task.this.isCancellable();
            }

            @Override
            public Set<Object> getResources() {
                return unmodifiableResources;
            }
        };
    }

    default Task addResources(Object... resources) {
        return this.addResources(Set.of(resources));
    }

    default Task addResources(Set<Object> resources) {
        return new Task() {
            Set<Object> newResources;

            @Override
            public void init() {
                Task.this.init();
                newResources = new HashSet<>();
                newResources.addAll(resources);
                newResources = Collections.unmodifiableSet(newResources);
            }

            @Override
            public boolean update() {
                return Task.this.update();
            }

            @Override
            public boolean isCancellable() {
                return Task.this.isCancellable();
            }

            @Override
            public Set<Object> getResources() {
                return newResources;
            }
        };
    }

    default Task withOneshot(Runnable runnable) {
        return this.withUpdate(() -> {
            runnable.run();
            return true;
        });
    }

    default Task withContinuous(Runnable runnable) {
        return this.withUpdate(() -> {
            runnable.run();
            return false;
        });
    }

    default Task andThen(Task next) {
        boolean[] thisDone = {false};
        return new Task() {
            private Set<Object> resources;

            @Override
            public void init() {
                resources = new HashSet<>();
                Task.this.init();
                next.init();
                resources.addAll(Task.this.getResources());
                resources.addAll(next.getResources());
                resources = Collections.unmodifiableSet(resources);
            }

            @Override
            public boolean update() {
                // will run second immediately if first is done, kinda sketchy but removes overhead
                if (!thisDone[0]) {
                    thisDone[0] = Task.this.update();
                }
                if (thisDone[0]) {
                    return next.update();
                } else {
                    return false;
                }
            }

            @Override
            public boolean isCancellable() {
                return Task.this.isCancellable() && next.isCancellable();
            }

            @Override
            public Set<Object> getResources() {
                return resources;
            }
        };
    }

    default Task with(Task with) {
        boolean[] dones = {false, false};
        return new Task() {
            private Set<Object> resources;

            @Override
            public void init() {
                resources = new HashSet<>();
                Task.this.init();
                with.init();
                resources.addAll(Task.this.getResources());
                resources.addAll(with.getResources());
                resources = Collections.unmodifiableSet(resources);
            }

            @Override
            public boolean update() {
                if (!dones[0]) {
                    dones[0] = Task.this.update();
                }
                if (!dones[1]) {
                    dones[1] = with.update();
                }
                return dones[0] && dones[1];
            }

            @Override
            public boolean isCancellable() {
                return Task.this.isCancellable() && with.isCancellable();
            }

            @Override
            public Set<Object> getResources() {
                return resources;
            }
        };
    }

    default void runToCompletion() {
        this.init();
        while (!this.update()) ;
    }

}
