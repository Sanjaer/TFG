#include "fglove.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define MYPORT 3493    // Puerto al que conectarán los usuarios
#define BACKLOG 2      // Conexiones pendientes en cola


int sockfd; // El servidor escuchara por sockfd
int newfd; // las transferencias de datos se realizar mediante newfd
struct sockaddr_in my_addr; // contendra la direccion IP y el numero de puerto local
struct sockaddr_in their_addr;//Contendra la direccion IP y numero de puerto del cliente
socklen_t sin_size;//Contendra el tamaño de la escructura sockaddr_in
float datos[14];
int bucle=0;


void aceptar()//accept = Se utiliza en el servidor, con un socket habilitado para recibir conexiones
{
  sin_size = sizeof(struct sockaddr_in);

  //Se espera por conexiones

  if ((newfd = accept(sockfd, (struct sockaddr *)&their_addr,&sin_size)) == -1)
  {
    perror("accept");
    exit(1); // Si se produce un error se finaliza el programa
  }
  printf("server: conexion desde: %s\n", inet_ntoa(their_addr.sin_addr));
}


void enviar(float *datos)
{
  if (send(newfd, datos, 14*sizeof(float), 0) == -1){
    perror("send");
    exit(1);
  }
  else{
    printf("Enviando datos");
  }
}


void send_calibration(float *data_max, float *data_min){
  //concatenar ambos vectores, datamax + datamin
  //enviar la calibración
  //esperar a una respuesta positiva
  //terminar
}


int main (int argc, char *argv[]) {

  fdGlove *pglove = NULL; //puntero guante
  float glovevalues[18]; //array para guardar los valores
  unsigned short cal_data[18], cal_max[18], cal_min[18]; //arrays que guardan el máximo y mínimo para calibrar
  char* GLOVE_PORT = (char*)("/dev/usb/hiddev0");

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
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)//Crea el socket y verifica si hubo algun error
  {
    perror("socket");
    exit(1);
  }

  my_addr.sin_family = AF_INET; //Se sa un servidor Stream (Protocolo TCP)
  my_addr.sin_port = htons(MYPORT); //se asigna el puerto por el que se va a escuchar (3490)
  my_addr.sin_addr.s_addr = INADDR_ANY; // se usa la IP local
  bzero(&(my_addr.sin_zero), 8); // rellena con ceros el resto de la estructura

  if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
  {
    perror("bind");
    exit(1);
  }
  // Habilitamos el socket para recibir conexiones, con una cola de BACKLOG conexiones en espera como maximo
  if (listen(sockfd, BACKLOG) == -1)
  {
    perror("listen");
    exit(1);
  }

  //Glove values

  aceptar();

  // Send calibration
  send_calibration(cal_max, cal_min);

  bucle=0;

  while(1){
    fdGetSensorScaledAll(pglove,glovevalues);

    //calculo del volumen

    int volumen;

    if (glovevalues[FD_THUMBNEAR]>0.5 || glovevalues[FD_THUMBFAR]>0.5  || glovevalues[FD_INDEXNEAR]>0.5 || glovevalues[FD_INDEXFAR]>0.5 ||  glovevalues[FD_MIDDLENEAR]>0.5 || glovevalues[FD_MIDDLEFAR]>0.5 || glovevalues[FD_RINGNEAR]>0.5 || glovevalues[FD_RINGFAR]>0.5 || glovevalues[FD_LITTLENEAR]>0.5 || glovevalues[FD_LITTLEFAR]>0.5)
      {volumen=1;}
    else
      {volumen=2;}


    printf("\nPulgar1:%.2f,Pulgar2:%.2f, Pulgarindice:%.2f, indice1:%.2f,indice2:%.2f, indicecorazon:%.2f, corazon1:%.2f,corazon2:%.2f, corazonanular:%.2f, anular1:%.2f, anular2:%.2f, anularmenique:%.2f, meñique1:%.2f, meñique2:%.2f,volumen: %d, bucle:%d",glovevalues[FD_THUMBNEAR],glovevalues[FD_THUMBFAR],glovevalues[FD_THUMBINDEX],glovevalues[FD_INDEXNEAR],glovevalues[FD_INDEXFAR],glovevalues[FD_INDEXMIDDLE],glovevalues[FD_MIDDLENEAR],glovevalues[FD_MIDDLEFAR],glovevalues[FD_MIDDLERING],glovevalues[FD_RINGNEAR],glovevalues[FD_RINGFAR],glovevalues[FD_RINGLITTLE],glovevalues[FD_LITTLENEAR],glovevalues[FD_LITTLEFAR],volumen, bucle);

    datos[0]=glovevalues[FD_THUMBNEAR];
    datos[1]=glovevalues[FD_THUMBFAR];            
    datos[2]=glovevalues[FD_THUMBINDEX];
    datos[3]=glovevalues[FD_INDEXNEAR];
    datos[4]=glovevalues[FD_INDEXFAR];
    datos[5]=glovevalues[FD_INDEXMIDDLE];
    datos[6]=glovevalues[FD_MIDDLENEAR];
    datos[7]=glovevalues[FD_MIDDLEFAR];
    datos[8]=glovevalues[FD_MIDDLERING];
    datos[9]=glovevalues[FD_RINGNEAR];
    datos[10]=glovevalues[FD_RINGFAR];
    datos[11]=glovevalues[FD_RINGLITTLE];
    datos[12]=glovevalues[FD_LITTLENEAR]; 
    datos[13]=glovevalues[FD_LITTLEFAR];
    enviar(datos);
    bucle++;
    usleep(100000);

  }

//Closing glove
  int closeg = fdClose(pglove);
  if (closeg!=0)    {
    printf("\nGlove closed.\n\n");
  }
  else  {
    printf("\nError closing glove.\n");
    return -1;
  }
  close(newfd);

   return 0;
}
