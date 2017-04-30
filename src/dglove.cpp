/***      dglove.cpp
Developed by Lidia Santos
and adapted by Pablo San José                   
     License CC-BY-SA    ***/

#include "fglove.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define MYPORT 3493     // Port for client's connections
#define BACKLOG 1       // Pending connections (enqueued)
#define DATA_LEN 14     // Length of the data vector
#define PKT_SIZE sizeof(struct packet)  // Size of the packet


int sockfd; // Server socket (listening)
int newfd; // Connection socket, one for each client
struct sockaddr_in my_addr; // Server's IP and localport 
struct sockaddr_in their_addr;// Client's IP and localport
socklen_t sin_size;// sizeof(sockaddr_in)

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
Function that reserves memory for new packets PKT_SIZE ***/
packet* reserve_d_pkt(void){
  packet* new_pkt;
  new_pkt = (packet*)malloc(PKT_SIZE);

  if (new_pkt == NULL){
    printf("\nERROR AL RESERVAR MEMORIA PARA PKT\n");
  }
  return(new_pkt);
}


/***  sendCalibration
Sends first the values for the max calibration
and second the values for the min calibration ***/
void sendCalibration(unsigned short *cal_max, unsigned short *cal_min){
  printf("\nSending calibration data");
  packet *data_cal=NULL;

  data_cal = reserve_d_pkt();
  
  //Send data max
  data_cal->type = 'C'; // ASCII Code = 67

  for (int i = 0; i < DATA_LEN; i++){
    data_cal->data[i] = (float)cal_max[i];
    printf("\n%d", cal_max[i]);
  }

  if (send(newfd, data_cal, PKT_SIZE, 0) == -1){
    free(data_cal);
    error ((char *)"send", 2);
  }
  else{
    printf("\n^Sending MAX calibration data^");
  }

  //Send data min
  for (int i = 0; i < DATA_LEN; i++){
    data_cal->data[i] = (float)cal_min[i];
    printf("\n%d", cal_min[i]);
  }

  if (send(newfd, data_cal, PKT_SIZE, 0) == -1){;
    free(data_cal);
    error((char *)"send", 2);
  }
  else{
    printf("\n^Sending MIN calibration data^");
  }
  free(data_cal);

}

/*** accept_conn
Accepts new incoming connections ***/
void accept_conn(void){

  sin_size = sizeof(struct sockaddr_in);

  if ((newfd = accept(sockfd, (struct sockaddr *)&their_addr,&sin_size)) == -1)  {
    error((char *)"accept", 1);
  }
  printf("server: conexion desde: %s\n", inet_ntoa(their_addr.sin_addr));
  
}



int main (int argc, char *argv[]) {

  fdGlove *pglove = NULL; //pointer to the glove
  float glovevalues[18]; //array to save the values
  unsigned short cal_data[18], cal_max[14], cal_min[14]; //arrays for the calibration
  char* GLOVE_PORT = (char*)("/dev/usb/hiddev0");
  int optval; // flag value for setsockopt 
  packet *data = NULL; // Packet to be sent
  int count = 0; // counter
  int volumen;

  //Initialization of glove
  pglove = fdOpen(GLOVE_PORT);
  if (pglove == NULL) {
      printf("\nError. Glove not opened.\n");
      return -1;
  }

  //Glove Info
  printf("\nGlove info:");
  int glovetype = fdGetGloveType(pglove);
  if (glovetype==FD_GLOVE5U_USB)  {printf("\nGlove type:GLOVE5U_USB");}
  int glovehand = fdGetGloveHand(pglove);
  if (glovehand==FD_HAND_RIGHT)   {printf("\nGlove hand:RIGHT");}
  else    {printf("\nGlove hand:LEFT");}
  int numsensors = fdGetNumSensors(pglove);
  printf("\nNumber of sensors: %d",numsensors);


  //Calibration
  printf("Coloque la mano plana sobre una superficie, con todos los dedos pegados unos a otros");
  getchar();
  fdGetSensorRawAll(pglove,cal_data);
  cal_min[2] = cal_data[2]; // Thumb/Index - Minimum Abduction
  cal_min[3] = cal_data[3]; // Index Near - Minimum Flexure
  cal_min[4] = cal_data[4]; // Index Far - Minimum Flexure
  cal_min[5] = cal_data[5]; // Index/Middle - Minimum Abduction
  cal_min[6] = cal_data[6]; // Middle Near - Minimum Flexure
  cal_min[7] = cal_data[7]; // Middle Far - Minimum Flexure
  cal_min[8] = cal_data[8]; // Middle/Ring - Minimum Abduction
  cal_min[9] = cal_data[9]; // Ring Near - Minimum Flexure
  cal_min[10] = cal_data[10]; // Ring Far - Minimum Flexure
  cal_min[11] = cal_data[11]; // Ring/Little - Minimum Abduction
  cal_min[12] = cal_data[12]; // Little Near - Minimum Flexure
  cal_min[13] = cal_data[13]; // Little Far - Minimum Flexure


  printf("Cierre la mano con el pulgar hacia arriba");
  getchar();
  fdGetSensorRawAll(pglove,cal_data);
  cal_min[0] = cal_data[0]; // Thumb Near - Minimum Flexure
  cal_min[1] = cal_data[1]; // Thumb Far - Minimum Flexure
  cal_max[3] = cal_data[3]; // Index Near - Maximum Flexure
  cal_max[4] = cal_data[4]; // Index Far - Maximum Flexure
  cal_max[6] = cal_data[6]; // Middle Near - Maximum Flexure
  cal_max[7] = cal_data[7]; // Middle Far - Maximum Flexure
  cal_max[9] = cal_data[9]; // Ring Near - Maximum Flexure
  cal_max[10] = cal_data[10]; // Ring Far - Maximum Flexure
  cal_max[12] = cal_data[12]; // Little Near - Maximum Flexure
  cal_max[13] = cal_data[13]; // Little Far - Maximum Flexure


  printf("Extienda la mano con los dedos juntos y el pulgar sobre la palma");
  getchar();
  fdGetSensorRawAll(pglove,cal_data);
  cal_max[0] = cal_data[0]; // Thumb Near - Maximum Flexure
  cal_max[1] = cal_data[1]; // Thumb Far - Maximum Flexure


  printf("Extienda la mano sobre una superficie plana con los dedos separados");
  getchar();
  fdGetSensorRawAll(pglove,cal_data);
  cal_max[2] = cal_data[2]; // Thumb/Index - Maximum Abduction
  cal_max[5] = cal_data[5]; // Index/Middle - Maximum Abduction
  cal_max[8] = cal_data[8]; // Middle/Ring - Maximum Abduction
  cal_max[11] = cal_data[11]; // Ring/Little - Maximum Abduction


  fdSetCalibrationAll(pglove,cal_max, cal_min);

  //Socket setup
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
    error((char *)"socket", 1);
  }

  /* setsockopt: Handy debugging trick that lets 
   * us rerun the server immediately after we kill it; 
   * otherwise we have to wait about 20 secs. 
   * Eliminates "ERROR on binding: Address already in use" error. 
   */
  optval = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, 
       (const void *)&optval , sizeof(int));


  my_addr.sin_family = AF_INET; //IP v4 protocol
  my_addr.sin_port = htons(MYPORT); //assigns the port
  my_addr.sin_addr.s_addr = INADDR_ANY; // local IP
  bzero(&(my_addr.sin_zero), 8); // fills with zeros the structure

  if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1){
    error((char *)"bind", 1);
  }

  if (listen(sockfd, BACKLOG) == -1){
    error((char *)"listen", 1);
  }

  // Accept incoming connections
  accept_conn();

  // Send calibration
  sendCalibration(cal_max, cal_min);

  // Send data
  packet* data_send = NULL;
  int i;

  data_send = reserve_d_pkt();

  //Header for data
  data_send->type = 'D';

  while(1){
    fdGetSensorScaledAll(pglove,glovevalues);

    if (glovevalues[FD_THUMBNEAR]>0.5 || glovevalues[FD_THUMBFAR]>0.5  || glovevalues[FD_INDEXNEAR]>0.5 || glovevalues[FD_INDEXFAR]>0.5 ||  glovevalues[FD_MIDDLENEAR]>0.5 || glovevalues[FD_MIDDLEFAR]>0.5 || glovevalues[FD_RINGNEAR]>0.5 || glovevalues[FD_RINGFAR]>0.5 || glovevalues[FD_LITTLENEAR]>0.5 || glovevalues[FD_LITTLEFAR]>0.5)
      {volumen=1;}
    else
      {volumen=2;}

    // Skipping this line increases the speed
    printf("\nPulgar1:%.2f,Pulgar2:%.2f, Pulgarindice:%.2f, indice1:%.2f,indice2:%.2f, indicecorazon:%.2f, corazon1:%.2f,corazon2:%.2f, corazonanular:%.2f, anular1:%.2f, anular2:%.2f, anularmenique:%.2f, meñique1:%.2f, meñique2:%.2f,volumen: %d, count:%d",glovevalues[FD_THUMBNEAR],glovevalues[FD_THUMBFAR],glovevalues[FD_THUMBINDEX],glovevalues[FD_INDEXNEAR],glovevalues[FD_INDEXFAR],glovevalues[FD_INDEXMIDDLE],glovevalues[FD_MIDDLENEAR],glovevalues[FD_MIDDLEFAR],glovevalues[FD_MIDDLERING],glovevalues[FD_RINGNEAR],glovevalues[FD_RINGFAR],glovevalues[FD_RINGLITTLE],glovevalues[FD_LITTLENEAR],glovevalues[FD_LITTLEFAR],volumen, count);

    //Composition of the  packet
    data_send->data[0]=glovevalues[FD_THUMBNEAR];
    data_send->data[1]=glovevalues[FD_THUMBFAR];            
    data_send->data[2]=glovevalues[FD_THUMBINDEX];
    data_send->data[3]=glovevalues[FD_INDEXNEAR];
    data_send->data[4]=glovevalues[FD_INDEXFAR];
    data_send->data[5]=glovevalues[FD_INDEXMIDDLE];
    data_send->data[6]=glovevalues[FD_MIDDLENEAR];
    data_send->data[7]=glovevalues[FD_MIDDLEFAR];
    data_send->data[8]=glovevalues[FD_MIDDLERING];
    data_send->data[9]=glovevalues[FD_RINGNEAR];
    data_send->data[10]=glovevalues[FD_RINGFAR];
    data_send->data[11]=glovevalues[FD_RINGLITTLE];
    data_send->data[12]=glovevalues[FD_LITTLENEAR]; 
    data_send->data[13]=glovevalues[FD_LITTLEFAR];

    if (send(newfd, data_send, PKT_SIZE, 0) == -1){
      error((char *)"send", 3);
    }
    else{
      printf("\nSending data");

    }
    count++;
    usleep(100000);

  }

  free(data_send);
  close(newfd);

  //Closing glove
  int closeg = fdClose(pglove);
  if (closeg!=0)    {
    printf("\nGlove closed.\n\n");
  }
  else  {
    printf("\nError closing glove.\n");
    return -1;
  }

  return 0;
}
