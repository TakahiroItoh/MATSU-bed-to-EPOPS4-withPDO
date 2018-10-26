//------------------------------------------------------------
//for MATSU-bed to EPOS4 with "PDO" communication
//operating mode : "Profile Velocity Mode"
//MATSU-bedからSYNC信号を送り、PDO communicationを行う
//Created by Takahiro Itoh
//--------------------プログラムの流れ--------------------------
//1.SDOコマンドでCtrlWord(Reset => Shutdown => Enable)を送信
//2.NMTコマンドでPreOperationalからOperationalに変更
//3.RxPDO2にModes of Operationalを送信
//4.RxPDO3にTarget VelocityとCtrlWord(Enable)を送信,SYNC送信開始
//5.RxPDO1にCtrlWord(Halt,Shutdown)を送信
//-------------------------------------------------------------

#include "mbed.h"
#include "USBSerial.h"

#define LED1 P0_29
#define LED2 P0_28
#define LED3 P0_27
#define LED4 P0_26

#define RxPDO1 0x220
#define RxPDO2 0x320
#define RxPDO3 0x420

#define Halt 1
#define QuickStop 2
#define ShutDown 3

USBSerial pc;
char Serialdata;
BusOut myled(LED1, LED2, LED3,LED4);

//回転速度指定
int rpm = 4000;             //Velocity[rpm]

CANMessage canmsgTx;
//CANMessage canmsgRx;
CAN canPort(P0_13, P0_18);  //CAN name(PinName rd, PinName td)
Ticker SYNC;

//プロトタイプ宣言
//------------------NMT-------------------
void NMTPreOpn(void);
void NMTOpn(void);
//------------------SYNC-------------------
void sendSYNC(void);
//------------------PDO--------------------
void CtrlWord(int);         //RxPDO1
void ModesOfOperation(void);//RxPDO2
void TgtVelCtrl(int);       //RxPDO3
//------------------SDO--------------------
void sendCtrlRS(int);       //Reset
void sendCtrlSD(int);       //Shutdown
void sendCtrlEN(int);       //Enable
//-------------------その他--------------------
void initialize(int);
void printCANTX(void);      //CAN送信データをPCに表示
//void printCANRX(void);      //CAN受信データをPCに表示
//void CANdataRX(void);       //CAN受信処理
void SerialRX(void);        //Serial受信処理

int main(){
    //Serial Setting
    pc.attach(SerialRX);        //Serial受信割り込み開始
    //CAN Setting
    canPort.frequency(1000000); //Bit Rate:1MHz
    int node = 2;               //CAN node数

    myled = 0b0101;
    pc.printf("Press 's' to Start\r\n");
    while(1){
        if(Serialdata == 's'){
            break;
        }
        myled=~myled;
        wait(0.5);
    }
    Serialdata = 0;
    pc.printf("KEY DETECTED!!\r\nPROGRAM START\r\n");
    myled = 0b0011;
    wait(1);

    //node初期化
    initialize(node);
    myled = 0b0111;
//    canPort.attach(CANdataRX,CAN::RxIrq);  //CAN受信割り込み開始
    pc.printf("'m'=Mode set, 't'=TgtVel, 'h'=Halt, 'q'=END\r\n");
    SYNC.attach(&sendSYNC,0.02);
    myled = 0b1111;
    //-------------------------------------------
    while(1){
        myled =~ myled;
        wait(0.5);
        if(Serialdata == 'q')break;
    }
    //quick stopコマンド送信
    pc.printf("Send Quick Stop\r\nPROGRAM END\r\n");
    CtrlWord(QuickStop);
    wait(0.5);
    CtrlWord(ShutDown);
    wait(0.5);
    SYNC.detach();
    while (1) {
        myled = 0b0000;
        wait(0.5);
    }
}

//NMT
void NMTPreOpn(void){
    //COB-ID:0 0x01-00-//-//-//-//-//-//
    canmsgTx.id = 0x0;
    canmsgTx.len = 2;
    canmsgTx.data[0] = 0x80;//0x01:enter NMT state "PreOperational"
    canmsgTx.data[1] = 0x00;//send All nodes
    printCANTX();
    canPort.write(canmsgTx);
    wait(0.2);
}
void NMTOpn(void){
    //COB-ID:0 0x01-00-//-//-//-//-//-//
    canmsgTx.id = 0x0;
    canmsgTx.len = 2;
    canmsgTx.data[0] = 0x01;//0x01:enter NMT state "Operational"
    canmsgTx.data[1] = 0x00;//send All nodes
    printCANTX();
    canPort.write(canmsgTx);
    wait(0.2);
}
//SYNC
void sendSYNC(void){
    canmsgTx.id = 0x80;
    canmsgTx.len = 0;
    canPort.write(canmsgTx);
//    printCANRX();
}
//PDO
void CtrlWord(int type){
    canmsgTx.id = RxPDO1;
    canmsgTx.len = 2;       //Data Length
    if(type==Halt){
        canmsgTx.data[0] = 0x0F;//data:0x01"0F"
        canmsgTx.data[1] = 0x01;//data:0x"01"0F
    }
    else if(type==QuickStop){
        canmsgTx.data[0] = 0x0B;//data:0x00"0B"
        canmsgTx.data[1] = 0x00;//data:0x"00"0B
    }
    else if(type==ShutDown){
        canmsgTx.data[0] = 0x06;//data:0x00"06"
        canmsgTx.data[1] = 0x00;//data:0x"00"06
    }
    printCANTX();           //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
void ModesOfOperation(void){
    canmsgTx.id = RxPDO2;
    canmsgTx.len = 1;       //Data Length
    canmsgTx.data[0] = 0x03;//data:0x03 = "Profile Velocity Mode"
    printCANTX();           //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
void TgtVelCtrl(int rpm){
    pc.printf("%drpm|0x%08x\r\n",rpm,rpm);
    canmsgTx.id = RxPDO3;
    canmsgTx.len = 6;       //Data Length
    //Target Velocity
    for(char cnt=0;cnt<4;cnt++){
        canmsgTx.data[cnt] = rpm % 256;
        rpm = rpm / 256;
    }
    //CtrlWord Enable
    canmsgTx.data[4] = 0x0F;//data:0x00"0F"
    canmsgTx.data[5] = 0x00;//data:0x"00"0F
    printCANTX();           //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
}
//SDO
void sendCtrlRS(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x80;//data:0x00"80" = "Controlword(Reset)"
    canmsgTx.data[5] = 0x00;//data:0x"00"80
    printCANTX();           //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
    wait(0.2);
}
void sendCtrlSD(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x06;//data:0x00"06" = "Controlword(Shutdown)"
    canmsgTx.data[5] = 0x00;//data:0x"00"06
    printCANTX();           //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
    wait(0.2);
}
void sendCtrlEN(int nodeID){
    canmsgTx.id = 0x600+nodeID;
    canmsgTx.len = 6;       //Data Length
    canmsgTx.data[0] = 0x2B;//|0Byte:40|1Byte:2F|2Byte:2B|4Byte:23|other:22|
    canmsgTx.data[1] = 0x40;//Index LowByte
    canmsgTx.data[2] = 0x60;//Index HighByte
    canmsgTx.data[3] = 0x00;//sub-Index
    canmsgTx.data[4] = 0x0F;//data:0x00"0F" = "Controlword(Enable)"
    canmsgTx.data[5] = 0x00;//data:0x"00"0F
    printCANTX();           //CAN送信データをPCに表示
    canPort.write(canmsgTx);//CANでデータ送信
    wait(0.2);
}
//初期化
void initialize(int node){
    //SDOコマンドで各nodeをリセット
    for(int nodeID=1;nodeID <= node;nodeID++){
        pc.printf("Send Reset CtrlWord Command node:%d\r\n",nodeID);
        sendCtrlRS(nodeID);
        pc.printf("Send Shutdown Command node:%d\r\n",nodeID);
        sendCtrlSD(nodeID);
        pc.printf("Send Enable Command node:%d\r\n",nodeID);
        sendCtrlEN(nodeID);
        pc.printf("----------------------------------------\r\n");
    }
    //NMTコマンドで全nodeをオペレーショナルに
    pc.printf("Send NMT PreOperational Command\r\n");
    NMTPreOpn();
    pc.printf("Send NMT Operational Command\r\n");
    NMTOpn();
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
/*
//受信データの表示
void printCANRX(void){
    char num;
    num = canmsgRx.len;
    pc.printf("0x%3x|",canmsgRx.id);
    for(char i=0;i < canmsgRx.len;i++){
        num = num - 1;
        pc.printf("%02x",canmsgRx.data[num]);
    }
    pc.printf("\r\n");
}
//CAN受信割り込み処理
void CANdataRX(void){
    canPort.read(canmsgRx);
}
*/
//Serial受信割り込み処理
void SerialRX(void){
    Serialdata = pc.getc();
    pc.printf("%c\r\n",Serialdata);
    //-------------送信コマンドを選択--------------
    if(Serialdata == 't'){
        //pc.printf("Send RxPDO TgtVel-Enable\r\n");
        TgtVelCtrl(1000);
        Serialdata = 0;
    }
    else if(Serialdata == 'y'){
        TgtVelCtrl(2000);
        Serialdata = 0;
    }
    else if(Serialdata == 'u'){
        TgtVelCtrl(3000);
        Serialdata = 0;
    }
    else if(Serialdata == 'i'){
        TgtVelCtrl(500);
        Serialdata = 0;
    }
    else if(Serialdata == 'o'){
        TgtVelCtrl(0);
        Serialdata = 0;
    }
    else if(Serialdata == 'h'){
        //Haltコマンド送信
        pc.printf("Send Halt Command\r\n");
        CtrlWord(Halt);
        Serialdata = 0;
        myled = 0b0111;
    }
    else if(Serialdata == 'm'){
        pc.printf("Send Operating Mode\r\n");
        ModesOfOperation();
        myled = 0b0111;
        Serialdata = 0;
    }
}
