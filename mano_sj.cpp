#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
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
#include <sys/wait.h>
#include <mutex>

#define PORT 3493 // puerto al que vamos a conectar

using namespace std;

int sockfd, numbytes;
struct hostent *he;
struct sockaddr_in their_addr; // información de la dirección de destino

int longitud=14,j=0;
float datos[14];
float radindex1=0, radmiddle1=0, radring1=0, radlittle1=0, radthumb1=0,radindex2=0, radmiddle2=0;
float radring2=0, radlittle2=0, radthumb2=0, radindexmiddle=0, radmiddlering=0, radringlittle=0, radthumbindex=0;
bool prueba=true;
int a;

std::mutex mtx;           // mutex for critical section

int main (int argc, char **argv)
{
    //hostname
    if ((he=gethostbyname("127.0.0.1")) == NULL)
    {
        perror("gethostbyname");
        exit(1);
    }
    //Crear socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        exit(1);
    }
    //atributos
    int len = sizeof(struct sockaddr_in);
    their_addr.sin_family = AF_INET;    // Ordenación de bytes de la máquina
    their_addr.sin_port = htons(PORT);  // short, Ordenación de bytes de la red
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);// se pasa la direccion ip al socket
    memset(&(their_addr.sin_zero), 8, len);  // poner a cero el resto de la estructura
    //conectar
    if (connect(sockfd, (struct sockaddr *)&their_addr,sizeof(struct sockaddr)) == -1)
    {
        perror("connect");
        exit(1);
    }

    cpid = fork();
    if (cpid == -1) {
       perror("fork");
       exit(EXIT_FAILURE);
    }

    if (cpid == 0) {        /* Child for calibration */
        mtx.lock();
        // Receive calibration data
        // Transfer calibration data
        // Wait for positive response (set timeout)
        // Send response to calibration
        mtx.unlock();
        _exit(EXIT_SUCCESS);

    } else {                /* Parent for normal usage */

        wait(2);
        mtx.lock();

        ros::init(argc, argv, "glove");
        ros::NodeHandle nh;

        ros::Publisher indice1 = nh.advertise<std_msgs::Float64>("indice1", 100);
        ros::Publisher indice2 = nh.advertise<std_msgs::Float64>("indice2", 100);
        // ros::Publisher indicecorazon = nh.advertise<std_msgs::Float64>("indicecorazon", 100);
        ros::Publisher corazon1 = nh.advertise<std_msgs::Float64>("corazon1", 100);
        ros::Publisher corazon2 = nh.advertise<std_msgs::Float64>("corazon2", 100);
        // ros::Publisher corazonanular = nh.advertise<std_msgs::Float64>("corazonanular", 100);
        ros::Publisher anular1 = nh.advertise<std_msgs::Float64>("anular1", 100);
        ros::Publisher anular2 = nh.advertise<std_msgs::Float64>("anular2", 100);
        // ros::Publisher anularmenique = nh.advertise<std_msgs::Float64>("anularmenique", 100);
        ros::Publisher menique1 = nh.advertise<std_msgs::Float64>("menique1", 100);
        ros::Publisher menique2 = nh.advertise<std_msgs::Float64>("menique2", 100);
        // ros::Publisher pulgarindice = nh.advertise<std_msgs::Float64>("pulgarindice", 100);
        ros::Publisher pulgar1 = nh.advertise<std_msgs::Float64>("pulgar1", 100);
        ros::Publisher pulgar2 = nh.advertise<std_msgs::Float64>("pulgar2", 100);
        ros::Publisher expansion = nh.advertise<std_msgs::String>("expansion", 100);

        std_msgs::Float64 str_msgind1;
        std_msgs::Float64 str_msgind2;
        // std_msgs::Float64 str_msgindcor;
        std_msgs::Float64 str_msgcor1;
        std_msgs::Float64 str_msgcor2;
        // std_msgs::Float64 str_msgcoranu;
        std_msgs::Float64 str_msganu1;
        std_msgs::Float64 str_msganu2;
        // std_msgs::Float64 str_msganumen;
        std_msgs::Float64 str_msgmen1;
        std_msgs::Float64 str_msgmen2;
        // std_msgs::Float64 str_msgpulind;
        std_msgs::Float64 str_msgpul1;  
        std_msgs::Float64 str_msgpul2;
        std_msgs::String str_msgexp;

        ros::Rate loop_rate(10);
        int i=0;

        while (ros::ok()){
            if ((numbytes=recv(sockfd, datos, 14*sizeof(float), 0)) == -1) {
                perror("recv");
                exit(1);
            }
            else  {
                printf("\nPulgar1: %.2f,Pulgar2: %.2f, Pulgarindice: %.2f,indice1: %.2f,indice2: %.2f,indicemedio: %.2f, medio1: %.2f,medio2: %.2f, medioanular: %.2f, anular1: %.2f,anular2: %.2f,anularmenique: %.2f, meñique1: %.2f,meñique2: %.2f, bucle:%d",datos[0],datos[1],datos[2],datos[3],datos[4],datos[5],datos[6],datos[7],datos[8],datos[9],datos[10],datos[11],datos[12],datos[13],i);
                i++;
            }

            str_msgpul1.data = datos[0];
            pulgar1.publish(str_msgpul1);

            str_msgpul2.data = datos[1];
            pulgar2.publish(str_msgpul2);

            // str_msgpulind.data = datos[2];
            // pulgarindice.publish(str_msgpulind);

            str_msgind1.data = datos[3];
            indice1.publish(str_msgind1);

            str_msgind2.data = datos[4];
            indice2.publish(str_msgind2);

            // str_msgindcor.data =datos[5];
            // indicecorazon.publish(str_msgindcor);

            str_msgcor1.data = datos[6];
            corazon1.publish(str_msgcor1);

            str_msgcor2.data = datos[7];
            corazon2.publish(str_msgcor2);

            // str_msgcoranu.data = datos[8];
            // corazonanular.publish(str_msgcoranu);

            str_msganu1.data = datos[9];
            anular1.publish(str_msganu1);

            str_msganu2.data = datos[10];
            anular2.publish(str_msganu2);

            // str_msganumen.data = datos[11];
            // anularmenique.publish(str_msganumen);

            str_msgmen1.data = datos[12];
            menique1.publish(str_msgmen1 );

            str_msgmen2.data = datos[13];
            menique2.publish(str_msgmen2);

            str_msgexp.data = "";

            str_msgexp.data = str_msgexp.data + to_string(datos[2]).substr(0,6);    // Pulgar-índice
            str_msgexp.data = str_msgexp.data + to_string(datos[5]).substr(0,6);    // Indice-corazón
            str_msgexp.data = str_msgexp.data + to_string(datos[8]).substr(0,6);    // Conrazón-anular
            str_msgexp.data = str_msgexp.data + to_string(datos[11]).substr(0,6);   // Anular-meñique
            
            // cout << str_msgexp.data << "\n";
            expansion.publish(str_msgexp);


            ros::spinOnce();
            loop_rate.sleep();
            j++;
        }


        close(sockfd);

        return 0;
    }
}
