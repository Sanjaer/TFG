/***      mano.cpp
Developed by Lidia Santos
and adapted by Pablo San José                   
     License CC-BY-SA    ***/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <fstream>
#include <sys/wait.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>


#define PORT 3493 // puerto al que vamos a conectar
#define DATA_LEN 14 // length of the data vector
#define DATA_SIZE 14*sizeof(float) // size of data vector
#define PKT_SIZE sizeof(struct packet)  // size of the packet struct
#define SOCKADDR_IN_SIZE sizeof(struct sockaddr_in)
#define SOCKADDR_SIZE sizeof(struct sockaddr)

using namespace std;

int sockfd, numbytes;
struct hostent *he;
struct sockaddr_in their_addr; // informacion de la direccion de destino

/* Structure for the packet */
struct packet{
  char type;
  float data[14];
};

/***   error
Prints the given error and exits with the given status ***/
void error (char *errmsg, unsigned short errcode){
  perror(errmsg);
  exit(errcode);
}

/***  reserve_d_pkt
Function that reserves memory for new packets PKT_SIZE  ***/
packet* reserve_d_pkt(void){
    packet* new_pkt;
    new_pkt = (packet*)malloc(PKT_SIZE);

    if (new_pkt == NULL){
        printf("\nERROR AL RESERVAR MEMORIA PARA PKT\n");
    }

    return(new_pkt);   
}


int main (int argc, char **argv){

    packet *data_recv = NULL; // pointer to reserved memory for struct
    int counter=0;
    float fcal_max[14];
    float fcal_min[14];


    //hostname
    if ((he=gethostbyname("127.0.0.1")) == NULL){
        error((char *)"gethostbyname", 1);
    }

    //Create socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
        error ((char *)"socket", 1);
    }


    their_addr.sin_family = AF_INET;    
    their_addr.sin_port = htons(PORT);
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);
    memset(&(their_addr.sin_zero), 8, SOCKADDR_IN_SIZE);

    if (connect(sockfd, (struct sockaddr *)&their_addr, SOCKADDR_SIZE) == -1){
        error ((char *)"connect", 1);
    }

    // Setting up ROS connection and publishers
    ros::init(argc, argv, "glove");
    ros::NodeHandle nh;

    ros::Publisher calibration_max = nh.advertise<std_msgs::Float64MultiArray>("calibration_max",1);
    ros::Publisher calibration_min = nh.advertise<std_msgs::Float64MultiArray>("calibration_min",1);

    ros::Publisher indice1 = nh.advertise<std_msgs::Float64>("indice1", 100);
    ros::Publisher indice2 = nh.advertise<std_msgs::Float64>("indice2", 100);
    ros::Publisher corazon1 = nh.advertise<std_msgs::Float64>("corazon1", 100);
    ros::Publisher corazon2 = nh.advertise<std_msgs::Float64>("corazon2", 100);
    ros::Publisher anular1 = nh.advertise<std_msgs::Float64>("anular1", 100);
    ros::Publisher anular2 = nh.advertise<std_msgs::Float64>("anular2", 100);
    ros::Publisher menique1 = nh.advertise<std_msgs::Float64>("menique1", 100);
    ros::Publisher menique2 = nh.advertise<std_msgs::Float64>("menique2", 100);
    ros::Publisher pulgar1 = nh.advertise<std_msgs::Float64>("pulgar1", 100);
    ros::Publisher pulgar2 = nh.advertise<std_msgs::Float64>("pulgar2", 100);
    ros::Publisher expansion = nh.advertise<std_msgs::Float64MultiArray>("expansion", 100);

    std_msgs::Float64MultiArray str_cal_max;
    std_msgs::Float64MultiArray str_cal_min;

    std_msgs::Float64 str_msgind1;
    std_msgs::Float64 str_msgind2;
    std_msgs::Float64 str_msgcor1;
    std_msgs::Float64 str_msgcor2;
    std_msgs::Float64 str_msganu1;
    std_msgs::Float64 str_msganu2;
    std_msgs::Float64 str_msgmen1;
    std_msgs::Float64 str_msgmen2;
    std_msgs::Float64 str_msgpul1;  
    std_msgs::Float64 str_msgpul2;
    std_msgs::Float64MultiArray str_msgexp;

    ros::Rate loop_rate(10);
    int i=0;

    data_recv = reserve_d_pkt();

    while (ros::ok()){
        if ((numbytes=recv(sockfd, data_recv, PKT_SIZE, 0)) == -1) {
            error ((char *)"recv", 3);
        }
        
        // In case it's a calibration packet
        if(data_recv->type == 'C'){
            printf("\nCalibration mode");

            // prints the first packet 
            for (i=0; i<DATA_LEN; i++){
                fcal_max[i] = data_recv->data[i];
                printf("\ncal_max[%d]= %f", i, fcal_max[i]);
            }

            // Reads the second packet
            if ((numbytes=recv(sockfd, data_recv, PKT_SIZE, 0)) == -1) {
                error ((char *)"recv", 2);
            }

            // prints the second packet
            for (i=0; i<DATA_LEN; i++){
                fcal_min[i] = data_recv->data[i];
                printf("\ncal_min[%d]= %f", i, fcal_min[i]);
            }

            // creates the messages for ROS
            str_cal_max.data.clear();
            str_cal_min.data.clear();
            for (i=0; i<14; i++){
                str_cal_max.data.push_back(fcal_max[i]);
                str_cal_min.data.push_back(fcal_min[i]);
            }

        } else{

            // this line can be skipped for speed
            printf("\nPulgar1: %.2f,Pulgar2: %.2f, Pulgarindice: %.2f,indice1: %.2f,indice2: %.2f,indicemedio: %.2f, medio1: %.2f,medio2: %.2f, medioanular: %.2f, anular1: %.2f,anular2: %.2f,anularmenique: %.2f, meñique1: %.2f,meñique2: %.2f, bucle:%d",data_recv->data[0],data_recv->data[1],data_recv->data[2],data_recv->data[3],data_recv->data[4],data_recv->data[5],data_recv->data[6],data_recv->data[7],data_recv->data[8],data_recv->data[9],data_recv->data[10],data_recv->data[11],data_recv->data[12],data_recv->data[13],i);
            
            // creation of the messages for ROS
            str_msgpul1.data = data_recv->data[0];
            pulgar1.publish(str_msgpul1);

            str_msgpul2.data = data_recv->data[1];
            pulgar2.publish(str_msgpul2);

            str_msgind1.data = data_recv->data[3];
            indice1.publish(str_msgind1);

            str_msgind2.data = data_recv->data[4];
            indice2.publish(str_msgind2);

            str_msgcor1.data = data_recv->data[6];
            corazon1.publish(str_msgcor1);

            str_msgcor2.data = data_recv->data[7];
            corazon2.publish(str_msgcor2);

            str_msganu1.data = data_recv->data[9];
            anular1.publish(str_msganu1);

            str_msganu2.data = data_recv->data[10];
            anular2.publish(str_msganu2);

            str_msgmen1.data = data_recv->data[12];
            menique1.publish(str_msgmen1 );

            str_msgmen2.data = data_recv->data[13];
            menique2.publish(str_msgmen2);

            // Publish sensors between fingers
            str_msgexp.data.clear();
            str_msgexp.data.push_back(data_recv->data[2]);
            str_msgexp.data.push_back(data_recv->data[5]);
            str_msgexp.data.push_back(data_recv->data[8]);
            str_msgexp.data.push_back(data_recv->data[11]);
            
            expansion.publish(str_msgexp);

            // Publish calibration
            calibration_max.publish(str_cal_max);
            calibration_min.publish(str_cal_min);
        }
        


        ros::spinOnce();
        loop_rate.sleep();
        counter++;
    }

    free(data_recv);

    close(sockfd);

    return 0;
}
