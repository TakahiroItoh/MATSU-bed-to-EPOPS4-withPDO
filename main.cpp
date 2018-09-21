//-------------------------------------------------------
//for MATSU-bed to EPOS4 with "PDO" communication
//operating mode : "Profile Velocity Mode"
//MATSU-bedからSYNC信号を送り、PDO communicationを行う
//Created by Takahiro Itoh
//-------------------------------------------------------

#include "mbed.h"
#include "USBSerial.h"

#define LED1 P0_29
#define LED2 P0_28
#define LED3 P0_27
#define LED4 P0_26

USBSerial pc;
char Serialdata;
BusOut myled(LED1, LED2, LED3, LED4);

CANMessage canmsgTx;
CANMessage canmsgRx;
CAN canPort(P0_13, P0_18);  //CAN name(PinName rd, PinName td)
Ticker SYNC;

//プロトタイプ宣言
//------------------send関数-------------------
//NMT Message
void sendNMTOpn(void);
//SYNC Message
void sendSYNC(void);
//mode Setting
void sendOPMode(int);       //Operating Mode
//Control Word
void sendCtrlRS(int);       //Reset
void sendCtrlSD(int);       //Shutdown
void sendCtrlEN(int);       //Switch on & Enable
void sendCtrlQS(int);       //Quick Stop
void sendCtrlHL(int);       //Halt
//Velocity Setting
void sendTgtVel(int,int);   //Target Velocity
//-------------------その他--------------------
void printCANTX(void);      //CAN送信データをPCに表示
void printCANRX(void);      //CAN受信データをPCに表示
void CANdataRX(void);       //CAN受信処理
void SerialRX(void);        //Serial受信処理

int main(){
    //Serial
    pc.attach(SerialRX);
    //CAN
    canPort.frequency(1000000); //Bit Rate:1MHz
    canPort.attach(CANdataRX,CAN::RxIrq);
    int node1 = 1;  //CAN node Setting
    //User Setting
    int rpm = 4000; //Velocity Setting[rpm]
    myled = 0b0001;
    pc.printf("Press 's' to Start\r\n");
    while(1){
        if(Serialdata == 's'){
            Serialdata = 0;
            break;
        }
        myled = 0b0001;
        wait(0.5);
        myled = 0b0000;
        wait(0.5);
    }
    Serialdata = 0;
    pc.printf("KEY DETECTED!!\r\nPROGRAM START\r\n");
    wait(0.5);
    //-------------起動時に必ず送信---------------
    //オペレーティングモードを送信
    pc.printf("Send Operating Mode\r\n");
    sendOPMode(node1);
    myled = 0b0011;
    wait(0.5);
    //コントロールワードのリセット
    pc.printf("Send Reset Command\r\n");
    sendCtrlRS(node1);
    wait(0.1);
    //Shutdown,Enableコマンド送信｜リセット
    pc.printf("Send Shutdown Command\r\n");
    sendCtrlSD(node1);
    wait(0.1);
    pc.printf("Send SW on & Enable Command\r\n");
    sendCtrlEN(node1);
    myled = 0b0111;
    wait(0.5);

    //NMT State
    pc.printf("Send NMT Operational Command\r\n");
    sendNMTOpn();
    wait(0.5);

    pc.printf("Press 't'=TgtVel 'h'=Halt 'q'=END\r\n");
    pc.printf("if EPOS4 dose not work. Press 'm'(set mode once again)\r\n");
    //-------------------------------------------
    while(1){
        //-------------送信コマンドを選択--------------
        if(Serialdata == 't'){
            //目標速度を送信後、Enableコマンド送信
            pc.printf("Send Target Velocity\r\n");
            sendTgtVel(node1,rpm);
            SYNC.attach(&sendSYNC,0.01);
            Serialdata = 0;
            myled = 0b1111;
        }
        else if(Serialdata == 'h'){
            //Haltコマンド送信
            pc.printf("Send Halt Command\r\n");
            sendCtrlHL(node1);
            Serialdata = 0;
            myled = 0b0111;
        }
        else if(Serialdata == 'q'){
            //quick stopコマンド送信
            SYNC.detach();
            pc.printf("Send Quick Stop\r\nPROGRAM END\r\n");
            sendCtrlQS(node1);
            pc.printf("Send Shutdown Command\r\n");
            sendCtrlSD(node1);
            Serialdata = 0;
            break;
        }
        else if(Serialdata == 'm'){
            pc.printf("Send Operating Mode\r\n");
            sendOPMode(node1);
            myled = 0b0011;
            wait(0.1);
            //コントロールワードのリセット
            pc.printf("Send Reset Command\r\n");
            sendCtrlRS(node1);
            wait(0.1);
            //Shutdown,Enableコマンド送信｜リセット
            pc.printf("Send Shutdown Command\r\n");
            sendCtrlSD(node1);
            wait(0.1);
            pc.printf("Send SW on & Enable Command\r\n");
            sendCtrlEN(node1);
            myled = 0b0111;
            wait(0.1);
            Serialdata = 0;
            myled = 0b0111;
        }
        //-------------------------------------------
    }
    myled = 0b0000;
}

//COB-ID:0 0x01-00-//-//-//-//-//-//
void sendNMTOpn(void){
    canmsgTx.id = 0x0;
    canmsgTx.len = 2;
    canmsgTx.data[0] = 0x01;//0x01:enter NMT state "Operational"
    canmsgTx.data[1] = 0x00;//send All nodes
    printCANTX();
    canPort.write(canmsgTx);
}
void sendSYNC(void){
    canmsgTx.id = 0x0;
    canmsgTx.len = 0;
    printCANTX();
    canPort.write(canmsgTx);
}
//0x2F-6060-00-03-//-//-//
void sendOPMode(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 5;       //Data Length
    canmsgTx.data[0] = 0x2F;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x60;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x03;//data:0x03 = "Profile Velocity Mode"
    /*
    canmsgTx.data[5] = 0x00;//data:(user value)
    canmsgTx.data[6] = 0x00;//data:(user value)
    canmsgTx.data[7] = 0x00;//data:(user value)
    */
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//0x2B-6040-00-0000-//-//
void sendCtrlRS(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x80;//data:0x00"80" = "Controlword(Shutdown)"
    canmsgTx.data[5] = 0x00;//data:0x"00"80
    /*
    canmsgTx.data[6] = 0x00;//data:(user value)
    canmsgTx.data[7] = 0x00;//data:(user value)
    */
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//0x2B-6040-00-0006-//-//
void sendCtrlSD(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x06;//data:0x00"06" = "Controlword(Shutdown)"
    canmsgTx.data[5] = 0x00;//data:0x"00"06
    /*
    canmsgTx.data[6] = 0x00;//data:(user value)
    canmsgTx.data[7] = 0x00;//data:(user value)
    */
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//0x2B-6040-00-000F-//-//
void sendCtrlEN(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x0F;//data:0x00"0F" = "Controlword(Enable)"
    canmsgTx.data[5] = 0x00;//data:0x"00"0F
    /*
    canmsgTx.data[6] = 0x00;//data:(user value)
    canmsgTx.data[7] = 0x00;//data:(user value)
    */
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//0x2B-6040-00-000B-//-//
void sendCtrlQS(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x0B;//data:0x00"0B" = "Quick Stop"
    canmsgTx.data[5] = 0x00;//data:0x"00"0B
    /*
    canmsgTx.data[6] = 0x00;//data:(user value)
    canmsgTx.data[7] = 0x00;//data:(user value)
    */
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//0x2B-6040-00-010F-//-//
void sendCtrlHL(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x0F;//data:0x01"0F" = "Halt"
    canmsgTx.data[5] = 0x01;//data:0x"01"0F
    /*
    canmsgTx.data[6] = 0x00;//data:(user value)
    canmsgTx.data[7] = 0x00;//data:(user value)
    */
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//0x2B-60FF-00-[user data(4Byte)]
void sendTgtVel(int nodeID,int rpm){
    pc.printf("%drpm|0x%08x\r\n",rpm,rpm);  //回転数送信データの表示
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 8;       //Data Length
    canmsgTx.data[0] = 0x23;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0xFF;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    //下位から1Byteずつdataに格納
    for(char cnt=4;cnt<8;cnt++){
        canmsgTx.data[cnt] = rpm % 256;
        rpm = rpm / 256;
    }
    printCANTX();          //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
    wait(0.5);
    //send Enable
    pc.printf("Send Enable Command\r\n");
    sendCtrlEN(nodeID);
    wait(0.5);
}

//送信データの表示
void printCANTX(void){
  //0x canID|Byte0|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|Byte7|
    pc.printf("0x%3x|",canmsgTx.id);
    for(char i=0;i < canmsgTx.len;i++){
        pc.printf("%02x|",canmsgTx.data[i]);
    }
    pc.printf("\r\n");
}
//受信データの表示
void printCANRX(void){
  //0x canID|Byte0|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|Byte7|
    pc.printf("0x%3x|",canmsgRx.id);
    for(char i=0;i < canmsgRx.len;i++){
        pc.printf("%02x|",canmsgRx.data[i]);
    }
    pc.printf("\r\n");
}
//CAN受信割り込み処理
void CANdataRX(void){
    canPort.read(canmsgRx);
    printCANRX();
}
//Serial受信割り込み処理
void SerialRX(void){
    Serialdata = pc.getc();
    pc.printf("%c\r\n",Serialdata);
}
