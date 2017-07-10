
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kobuki_fleet_msgs/ManagerTaskAction.h>


class TaskAction
{

  enum TaskStatus {
    OPEN=0,
    WORKING=1,
    FINISHED=2,
    ERROR=3};

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kobuki_fleet_msgs::ManagerTaskAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  kobuki_fleet_msgs::ManagerTaskFeedback feedback_;
  kobuki_fleet_msgs::ManagerTaskResult result_;
  unsigned int taskStatus_;


public:

  TaskAction(std::string name) :
    as_(nh_, name, boost::bind(&TaskAction::executeCB, this, _1), false),
    action_name_(name),
    taskStatus_(OPEN)
  {
    ROS_INFO("%s: Initialized", action_name_.c_str());
    as_.start();
  }

  ~TaskAction(void)
  {

  }

  void executeCB(const kobuki_fleet_msgs::ManagerTaskGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.task_status = taskStatus_;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, current task status: %i", action_name_.c_str(), taskStatus_);

    // start executing the action
    while(taskStatus_ == WORKING && nh_.ok())
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.task_status = taskStatus_;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_);
      // set the action state to succeeded

      feedback_.task_status = taskStatus_;
      as_.publishFeedback(feedback_);
      result_.task_status = taskStatus_;
      as_.setSucceeded(result_);
    }
  }

  void setStatus(int status)
  {
    ROS_INFO("old status: %i  new status: %i", taskStatus_, status);
    taskStatus_ = status;
  }

};


/*int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_server");
  //ros::NodeHandle nh_;
  TaskAction task_manager("task_action_server");
  task_manager.setStatus(1);
  ros::spin();

  return 0;
}*/
