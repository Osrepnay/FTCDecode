package org.firstinspires.ftc.teamcode.noncents.tasks;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class TaskRunner {
    List<Task> waitingTasks = new ArrayList<>();
    List<Task> runningTasks = new ArrayList<>();
    Set<Object> consumedResources = new HashSet<>();

    public boolean sendTask(Task task) {
        task.init();
        Set<Object> taskResources = task.getResources();
        Set<Object> conflictingResources = new HashSet<>(taskResources);
        conflictingResources.retainAll(consumedResources);
        if (conflictingResources.isEmpty()) {
            runningTasks.add(task);
            consumedResources.addAll(taskResources);
            return true;
        } else {
            List<Integer> tasksToRemove = new ArrayList<>();
            boolean allCancellable = true;
            for (int i = 0; i < runningTasks.size(); i++) {
                Task runningTask = runningTasks.get(i);
                if (!Collections.disjoint(runningTask.getResources(), conflictingResources)) {
                    if (!runningTask.isCancellable()) {
                        allCancellable = false;
                        break;
                    } else {
                        tasksToRemove.add(i);
                    }
                }
            }
            if (allCancellable) {
                for (int i = tasksToRemove.size() - 1; i >= 0; i--) {
                    consumedResources.removeAll(runningTasks.get(tasksToRemove.get(i)).getResources());
                    runningTasks.remove((int) tasksToRemove.get(i));
                }
                runningTasks.add(task);
                consumedResources.add(task.getResources());
                return true;
            } else {
                waitingTasks.add(task);
                return false;
            }
        }
    }

    public void update() {
        for (int i = 0; i < runningTasks.size(); i++) {
            if (runningTasks.get(i).update()) {
                consumedResources.removeAll(runningTasks.get(i).getResources());
                runningTasks.remove(i);
                i--;
            }
        }
        List<Task> waitingTasksCopy = new ArrayList<>(waitingTasks);
        waitingTasks.clear();
        for (Task task : waitingTasksCopy) {
            // hacky but works [citation needed]
            // TODO change anyway, introduces more overhead and can't deal with canceling tasks started here
            // TOOD also causes double init
            this.sendTask(task);
        }
    }

    public void flush() {
        waitingTasks.clear();
        runningTasks.clear();
        consumedResources.clear();
    }
}
