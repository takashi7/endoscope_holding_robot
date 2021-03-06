//#include <stdio.h>
//#include <string.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <unistd.h>

////int main()
////{
//// int sock;
//// struct sockaddr_in addr;

//// char buf[2048];

//// sock = socket(AF_INET, SOCK_DGRAM, 0);

//// addr.sin_family = AF_INET;
//// addr.sin_port = htons(35000);
//// addr.sin_addr.s_addr = INADDR_ANY;

//// bind(sock, (struct sockaddr *)&addr, sizeof(addr));

//// memset(buf, 0, sizeof(buf));
//// recv(sock, buf, sizeof(buf), 0);

//// printf("%s\n", buf);

//// close(sock);

//// return 0;
////}


//typedef struct {
//	int sequence;
//	int w[4];
//}Data;

//int main()
//{
//  int port = 35000;
//  struct sockaddr_in me;
//  memset( &me, 0, sizeof(me) );
//  me.sin_family = AF_INET;
//  me.sin_addr.s_addr = htonl( INADDR_ANY );
//  me.sin_port = htons(port);
//  
//  int sock = socket(AF_INET, SOCK_DGRAM, 0);
//  
//  int ret = bind(sock, (struct sockaddr *)&me, sizeof(me) );
//  if( ret<0 ){
//    perror("bind");
//    return -1;
//  }
// 
//  int count =0;
//  Data buf;
// //-----------------------------------------------------------------



//  while(1){


////------ 制御ループの中にコピペ--------------------------------
//    struct timeval tv={0,10};
//    fd_set mask;
//    FD_ZERO( &mask );
//    FD_SET( sock, &mask );
//    int ret = select( 20, &mask, NULL, NULL, NULL ); 
//    
//    if( FD_ISSET(sock, &mask) ){
//      
//      struct sockaddr_in from;
//      int fromlen = sizeof(from);
//      
//      recvfrom(sock, (char*)&buf, sizeof(buf), 0, NULL, 0);
//      float pri[4];
//      pri[0] = buf.w[0]/1000;
//      pri[1] = buf.w[1]/1000;
//      pri[2] = buf.w[2]/1000;
//      pri[3] = buf.w[3]/1000;

//      printf("%6d, [%f, %f, %f, %f]\n", 
//	     buf.sequence,
//	     pri[0], pri[1], pri[2], pri[3]
//	     );
////     printf("%6d, [%d, %d, %d, %d]\n", 
////	     buf.sequence,
////	     buf.w[0], buf.w[1], buf.w[2], buf.w[3]
////	     );

//      count++;
//	}
////-----------------------------------------------------------

//  }


//  //--- 後処理 ------
//  close(sock);
//  //-----------------

//  return 0;
//}


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
	ros::init(argc, argv, "real_master");
	ros::NodeHandle node;
	ros::Publisher caemera_pose = node.advertise<geometry_msgs::Twist>("real/master", 10); 
	// /ROS  
	//-----------------------------------------------------------------------------------------
	  
	//-----------------------------------------------------------
	// UDP
	int port = 35000;
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
	Data buf;
	// /UDP
  	//-----------------------------------------------------------

	//--------------------------------------------------------------------------------------
  	// Loop
	ros::Rate rate(100.0);
	while (node.ok()){
		//------------------------------------------------------------------------------
		// UDP
		struct timeval tv={0,10};
		fd_set mask;
		FD_ZERO( &mask );
		FD_SET( sock, &mask );
		int ret = select( 20, &mask, NULL, NULL, NULL );    
		if( FD_ISSET(sock, &mask) ){      
			struct sockaddr_in from;
			int fromlen = sizeof(from);      
			recvfrom(sock, (char*)&buf, sizeof(buf), 0, NULL, 0);
			float w_float[4];
			w_float[0] = buf.w[0]/1000;
			w_float[1] = buf.w[1]/1000;
			w_float[2] = buf.w[2]/1000;
			w_float[3] = buf.w[3]/1000;
			printf("%6d, [%f, %f, %f, %f]\n", 
				buf.sequence,
				w_float[0], w_float[1], w_float[2], w_float[3]
			);
			count++;
		// /USP
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
		rate.sleep();
		// /ROS
		//------------------------------------------------------------------------------  
	}
	// /Loop
	//--------------------------------------------------------------------------------------
	
	//--- 後処理 ------
	close(sock);
	//-----------------
	return 0;
}


