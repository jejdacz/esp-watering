#include "Arduino.h"

#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#ifndef FIRSTRUN_SEED 
#define FIRSTRUN_SEED 125
#endif

#ifndef TASKLIST_SIZE
#define TASKLIST_SIZE 5
#endif

typedef struct
  {
    void (*run)();
    uint32_t startTime;
  } task;

class TaskManager {

  private:    
    task _taskList[TASKLIST_SIZE];
    
    void initTaskList();
    void checkTaskList(uint32_t time);
        
  public:
    TaskManager();
    void handle(uint32_t time);
    task* getTaskList();
    int getTaskListByteSize();
    void addTask(void (*fn)(), uint32_t startTime);
    uint32_t nextTaskTime(uint32_t time);
          
};

#endif // TASKMANAGER_H