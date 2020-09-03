/*
 *
 */

#include "TaskManager.h"

namespace
{
  void nullFn(){};
  task nullTask{&nullFn, 0xffffffff};
} // namespace

//static void nullFn(){};
//static task nullTask{&nullFn, 0xffffffff};

TaskManager::TaskManager()
{
  this->initTaskList();
}

void TaskManager::handle(uint32_t time)
{
  this->checkTaskList(time);
}

task *TaskManager::getTaskList()
{
  return this->_taskList;
}

int TaskManager::getTaskListByteSize()
{
  return sizeof(this->_taskList);
}

void TaskManager::addTask(void (*fn)(), uint32_t startTime)
{
  int i = 0;
  while (i < TASKLIST_SIZE)
  {
    if (this->_taskList[i].startTime == 0xffffffff)
    {
      this->_taskList[i].run = fn;
      this->_taskList[i].startTime = startTime;
      break;
    }
    i++;
  }
}

void TaskManager::checkTaskList(uint32_t time)
{
  for (int i = 0; i < TASKLIST_SIZE; i++)
  {
    /*
    Serial.print("checking task index: ");
    Serial.print(i);
    Serial.print(" startTime: ");
    Serial.println(this->_taskList[i].startTime);
    */
    if (time >= this->_taskList[i].startTime)
    {
      // save task func pointer
      void (*taskFunc)() = this->_taskList[i].run;
      // remove task
      this->_taskList[i] = nullTask;
      // run task
      (*taskFunc)();
    }
  }
}

uint32_t TaskManager::nextTaskTime(uint32_t time)
{
  uint32_t t = 0xffffffff;
  for (int i = 0; i < TASKLIST_SIZE; i++)
  {
    t = min(this->_taskList[i].startTime, t);
    t = max(time, t);
  }
  return t;
}

void TaskManager::initTaskList()
{
  for (int i = 0; i < TASKLIST_SIZE; i++)
  {
    this->_taskList[i] = nullTask;
  }
}
