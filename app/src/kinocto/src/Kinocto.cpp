#include <Kinocto.h>

using namespace std;
using namespace ros;

Kinocto::Kinocto() {
	state = INITIATED;
}

Kinocto::~Kinocto() {
}

void Kinocto::start() {
	loop();
}

void Kinocto::loop() {
	while (ros::ok()) {
		switch (state) {
		case INITIATED:
			cout << "waiting" << endl;
			break;
		case START_LOOP:
			cout << "looping" << endl;
			break;
		}

		ros::spinOnce();
	}
}

bool Kinocto::startLoop(kinocto::StartKinocto::Request & request, kinocto::StartKinocto::Response & response) {
	state = START_LOOP;
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kinocto");
	ros::NodeHandle nodeHandle;

	Kinocto kinocto;

	ROS_INFO("%s", "Subscriving callback");
	ros::ServiceServer service = nodeHandle.advertiseService("start_kinocto", &Kinocto::startLoop, &kinocto);

	ROS_INFO("%s", "Kinocto Initiated");
	kinocto.start();

	return 0;
}
