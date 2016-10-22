
#include "telemetry.h"

#include <pgmspace.h>
#include <ESP8266WiFi.h>


//const char* ssid = "ONOFBA9";
//const char* password = "4125905522uceda";
const char* ssid = "wifiVictor";
const char* password = "wifivictor";


WiFiServer server(23);
WiFiClient serverClient;

#define SIZE_TELEMETRIA 120//40*30 30seg telemetry
#define DIM_TELEMETRIA 8
float telemetria[SIZE_TELEMETRIA+1][DIM_TELEMETRIA]={0};
int p_telemetria=0;


void init_telnet(){
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  server.begin();
  server.setNoDelay(true);
  Serial.print("\nReady! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");
}

void print_tl(String str){
  if (serverClient && serverClient.connected()){
        serverClient.print(str);
   }
}

void check_send_telnet_telemetry(){
  if (server.hasClient()){
    if (!serverClient || !serverClient.connected()){
        if(serverClient) serverClient.stop();
        serverClient = server.available();
        Serial.println("New client: ");
      }
  }

  //check clients for data
  /*if (serverClient && serverClient.connected()){
    if(serverClient.available()){
      //get data from the telnet client
      while(serverClient.available()){
          serverClient.read();
      }
    }
  }*/

//Send data
  if (serverClient && serverClient.connected()){
    for(int i=0;i<p_telemetria;i++){
      for(int j=0;j<DIM_TELEMETRIA;j++){
        serverClient.print(telemetria[i][j]);
        serverClient.print(" ");
      }
      serverClient.println("");
    }
    serverClient.println("");
    p_telemetria=0;
    delay(5);
   }

}
