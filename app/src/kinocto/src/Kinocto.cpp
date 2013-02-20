#include <Kinocto.h>

using namespace std;

Kinocto kinocto;

Kinocto::Kinocto () {
    state = INITIATED;
    ROS_INFO("%s", "Kinocto Initiated");
}

void Kinocto::loop() {
    for (;;) {
        switch (state) {
            case INITIATED : cout << "waiting" << endl;
            break;
            case START_LOOP : cout << "looping" << endl;
            break;
        }
        sleep(10);
    }
}

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Im starting the loop: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinocto");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
