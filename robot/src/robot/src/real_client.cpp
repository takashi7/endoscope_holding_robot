#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

void clientCallback(const geometry_msgs::Twist objectTwist);

typedef struct {
	int sequence;
	int w[4];
}Data;

int count = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "real_client");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("camera/twist/compare", 10, clientCallback);
	ros::spin();
	  
	return 0;
}

void clientCallback(const geometry_msgs::Twist objectTwist)
{
	int sock;
	struct sockaddr_in addr;

	sock = socket(AF_INET, SOCK_DGRAM, 0);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(35000);
	addr.sin_addr.s_addr = inet_addr("192.168.1.50");
	//addr.sin_addr.s_addr = inet_addr("192.168.10.143");
	 
	Data buf;
	buf.sequence = count;

	buf.w[0] = (int)(1000*objectTwist.angular.x);
	buf.w[1] = (int)(1000*objectTwist.angular.z);
	buf.w[2] = (int)(-1000*objectTwist.angular.y);
	buf.w[3] = (int)(1000*objectTwist.linear.z);
	 
	printf("%6d, [%d, %d, %d, %d]\n", 
		buf.sequence,
		buf.w[0], buf.w[1], buf.w[2], buf.w[3]
	);
	 
	++count;
	//printf("count: %d\n", count);

	 
	sendto(sock, (const char*)&buf, sizeof(buf), 0, (struct sockaddr *)&addr, sizeof(addr));

	close(sock);
}
