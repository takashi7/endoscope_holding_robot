#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>


typedef struct {
	int sequence;
	int w[4];
}Data;

int main(int argc, char **argv)
{
	//-----------------------------------------------------------------------------------------
	// ROS
	ros::init(argc, argv, "asparagus_master");
	ros::NodeHandle node;
	ros::Publisher caemera_pose = node.advertise<geometry_msgs::Twist>("asparagus/twist", 10); 
	// /ROS  
	//-----------------------------------------------------------------------------------------
	  
	//-----------------------------------------------------------
	// UDP
	int port = 35500;
	struct sockaddr_in me;
	memset( &me, 0, sizeof(me) );
	me.sin_family = AF_INET;
	me.sin_addr.s_addr = htonl( INADDR_ANY );
	me.sin_port = htons(port);  
	int sock = socket(AF_INET, SOCK_DGRAM, 0);  
	int ret = bind(sock, (struct sockaddr *)&me, sizeof(me) );
	if( ret<0 ){
		perror("bind");
		return -1;
		} 
	int count =0;
	Data buf_konno;
	// /UDP
  	//-----------------------------------------------------------

	//--------------------------------------------------------------------------------------
  	// Loop
  	printf("ok\n");  
	//ros::Rate rate(100.0);
	while (node.ok()){
		//------------------------------------------------------------------------------
		// UDP
		struct timeval tv={0,10};
		fd_set mask;
		FD_ZERO( &mask );
		FD_SET( sock, &mask );
		int ret = select( 20, &mask, NULL, NULL, NULL );  // 問題が起きてる
		printf("ok\n");				   
		if( FD_ISSET(sock, &mask) ){  
			struct sockaddr_in from;
			int fromlen = sizeof(from);      
			recvfrom(sock, (char*)&buf_konno, sizeof(buf_konno), 0, NULL, 0);
			printf("%d\n", buf_konno.w[3]);
			float w_float[4];
			w_float[0] = buf_konno.w[0]/1000;
			w_float[1] = buf_konno.w[1]/1000;
			w_float[2] = buf_konno.w[2]/1000;
			w_float[3] = buf_konno.w[3];			
			printf("%6d, [%f, %f, %f, %f]\n", 
				buf_konno.sequence,
				w_float[0], w_float[1], w_float[2], w_float[3]
			);
			//count++;
		// /UDP
		//------------------------------------------------------------------------------
			     
		//------------------------------------------------------------------------------
		// ROS
			geometry_msgs::Twist objectTwist;    
			objectTwist.angular.x = w_float[0];
			objectTwist.angular.y = w_float[1];
			objectTwist.angular.z = w_float[2];			    
			objectTwist.linear.z = w_float[3];

			caemera_pose.publish(objectTwist); 		
		}
		//rate.sleep();
		// /ROS
		//------------------------------------------------------------------------------  	
	}
	printf("end loop\n");
	// /Loop
	//--------------------------------------------------------------------------------------
	
	//--- 後処理 ------
	close(sock);
	//-----------------
	return 0;
}
