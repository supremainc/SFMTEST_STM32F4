/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

#include "UF_SysParameter.h"
#include "UF_API.h"

#include "UF_Packet.h"
#include "UF_Error.h"
#include "UF_Command.h"

#include "UF_Enroll.h"

/* USER CODE BEGIN Includes */

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(int chr)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

#define BinaryToAscii( data ) ( ( data ) >= 10 ? ( data ) - 10 + 'A' : ( data ) + '0' )
#define AsciiToBinary( data ) ( ( data ) >= 'A' ? ( data ) - 'A' + 10 : ( data ) - '0' )
char hextable[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
#define TOHEX(a, b)     {*b++ = hextable[a >> 4];*b++ = hextable[a&0xf];}

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

PUTCHAR_PROTOTYPE
{  
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    //HAL_UART_Transmit_IT(&huart2, (uint8_t *)&ch, 1);
    return ch;
}

GETCHAR_PROTOTYPE
{
    uint8_t inputChar;
    HAL_UART_Receive(&huart2, (uint8_t *)&inputChar, 1, 0xFFFFFFFF);
    //HAL_UART_Receive_IT(&huart2, (uint8_t *)&inputChar, 1);
    return inputChar;
}

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SysParameter *sysParameter;

char sysParamList[][20]={
    "TIMEOUT",
    "ENROLL_MODE",
    "SECURITY_LEVEL",
    "ENCRYPTION_MODE",
    "SENSOR_TYPE",
    "IMAGE_FORMAT",
    "MODULE_ID",
    "FIRMWARE_VERSION", //7
    "SERIAL_NUMBER", //8
    "BAUDRATE",
    "AUX_BAUDRATE",
    "ENROLLED_FINGER",
    "AVAILABLE_FINGER",
    "SEND_SCAN_SUCCESS",
    "ASCII_PACKET",
    "ROTATE_IMAGE",
    "SENSITIVITY",
    "IMAGE_QUALITY",
    "AUTO_RESPONSE",
    "NETWORK_MODE",
    "FREE_SCAN",
    "PROVISIONAL_ENROLL",
    "PASS_WHEN_EMPTY",
    "RESPONSE_DELAY",
    "MATCHING_TIMEOUT",
    "BUILD_NO", //25
    "ENROLL_DISPLACEMENT",
    "TEMPLATE_SIZE",
    "ROTATION",
    "LIGHTING_CONDITION",
    "FREESCAN_DELAY",
    "CARD_ENROLL_MODE",
    "FAST_MODE",
    "WATCHDOG",
    "TEMPLATE_TYPE",
    "ENHANCED_PRIVACY",
    "FAKE_DETECT",
    "CHECK_LATENT",
    "VOLTAGE_WARNING",
    "POWEROFF_TIMEOUT",
    "EXPOSURE",
    "RESERVED"
};

typedef struct _command
{
    char strCommand[4];
    int nCommand;
}UF_CommandList;

UF_CommandList commandParamList[]=
{
    {"SR",0x03},
    {"ES",0x05},
    {"VS",0x08},
    {"IS",0x11},
    {"DA",0x17},
    {"-1",-1}
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static int MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

#if 0
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
#endif

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];

//global variable


int g_nPort = COMPORT1;
int g_nBaudrate = 115200;
int g_nProtocol = 0;  //0 : single   1: network
int g_nModuleID = 1;
int g_bConnectionStatus = 0;
int g_bTrace = 0;

/* USER CODE END 0 */

#if 0
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USB_DEVICE_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();

    /* USER CODE BEGIN 2 */
    unsigned char send_buf[UF_PACKET_LEN]={0x40,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x0A};
    unsigned char recv_buf[UF_PACKET_LEN]="";
    /* USER CODE END 2 */

    HAL_UART_Receive_IT(&huart1, recv_buf, UF_PACKET_LEN);
    HAL_UART_Transmit_IT(&huart1, send_buf, UF_PACKET_LEN);

    printf("\r[RX] ");
    for (int i=0; i<13;i++)
    {
        HAL_Delay(4);   // The delay time may need to be changed.
        printf("%02X ", recv_buf[i]);
    }
    printf("\n");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}
#endif
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
#if 0
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    MX_USB_DEVICE_Init();
#endif
    MX_USART2_UART_Init(); // for debugging

    StartProgram();

    return 0;
}

int StartProgram(void)
{
    char strMenu[1];
    int nMenu;
    int bExit = 0;
    int RcvStat;

    while(bExit != 1)
    {
        MainMenu();

#if 0
        RcvStat = HAL_UART_Receive(&huart2, (uint8_t *)strMenu, 1, 1000);
//        RcvStat = HAL_UART_Receive_IT(&huart2, strMenu, 1);
        if (RcvStat == HAL_OK)
        {
            nMenu = atoi(strMenu);
            printf("%s\n", strMenu);
        }
#else
        fflush(stdin);
        strMenu[0] = getchar();
        nMenu = atoi(strMenu);
#endif
        switch(nMenu)
        {
            case 0:
                Commander();
                break;
            case 1:
                CommunicationSetup();
                break;
            case 2:
                Enroll();
                break;
            case 3:
                Identify();
                break;
            case 4:
                Verify();
                break;
            case 5:
                SystemParameterSetting();
                break;
            case 6:
                Delete_all();
                break;
            case 7:
                break;
            default:
                printf("Please, try again!\n");
                fflush(stdin);
                getchar();
                break;
        }
        nMenu = -1;
    }
    return 0;
}

void MainMenu(void)
{
    Intro();
    CurrentCummunicationSetup();
    if(g_bConnectionStatus)
      printf("\r\n0.Commander\n");
    printf("\r\n1.Communication Setup\n");
    printf("\r\n2.Enroll\n");
    printf("\r\n3.Identify\n");
    printf("\r\n4.Verify\n");
    printf("\r\n5.Sytem Parameter Setting\n");
    printf("\r\n6.Delete All\n");
    printf("\r\n7.Exit\n");
    printf("\r\n>>");
}

void Intro(void)
{
    printf("\r(%d)******************************\n", HAL_GetTick());
    printf("\r**********************************\n");
    printf("\r UFCommander v1.0 for STM32F4x \n");
    printf("\r**********************************\n");
}

void CommunicationSetupMenu(void)
{
    CurrentCummunicationSetup();
    printf("\r1.Connect\n");
    printf("\r2.Disconnect\n");
    printf("\r3.Reconnect\n");
    printf("\r4.Search Module\n");
    printf("\r5.Change Communication Setup\n");
    printf("\r6.Back\n");
    printf("\r>>");
}

void CurrentCummunicationSetup(void)
{
    printf("\r----------------------------------\n");
    printf("\r  Current Communication Setting   \n");
    printf("\r----------------------------------\n");

    printf("\r%-11s : %dbps\n", "Baudrate",huart1.Init.BaudRate);

    if(g_nProtocol == 0)
        printf("\r%-11s : Single\n","Protocol");
    else
        printf("\r%-11s : Network\n","Protocol");

    if(g_nProtocol == 1)
        printf("\r%-11s : %d\n","ModuleID", g_nModuleID);

    if(g_bConnectionStatus == 0)
        printf("\r%s\n","Status : [ Disconnected ]");
    else
        printf("\r%s\n","Status : [ Connected ]");


    if(sysParameter!=NULL && g_bConnectionStatus!=0)
    {
        printf("\r----------------------------------\n");
        printf("\r   Connected module information   \n");
        printf("\r----------------------------------\n");
        printf("\r%-11s : %c%c%c%c\n","F/W version", sysParameter[7].value>>24 & 0xff,sysParameter[7].value>>16 & 0xff,sysParameter[7].value>>8 & 0xff,sysParameter[7].value &0xff );
        printf("\r%-11s : %d\n", "Serial No.",sysParameter[8].value);
        printf("\r%-11s : %02x%02x%02x%02x\n","Build No.", sysParameter[25].value>>24 & 0xff, sysParameter[25].value>>16 & 0xff,sysParameter[25].value>>8 & 0xff,sysParameter[25].value &0xff );
    }

    printf("\r----------------------------------\n");
}

void Commander(void)
{
    if(g_bConnectionStatus == 0)
    {
        printf("\rThe module was not connected. Please, Connect to module first.\n");
        WaitKey();
        return;
    }

    printf("\rEnter the command and parameters.\n");
    printf("\rex: (SR 0x0 0x0 0x6d) or (0x3 0x0 0x0 0x6d)\n");
    printf("\r    (3 0 0 109)\n");
    printf("\r    (3 0 0 0x6d)\n");
    printf("\rType 'exit' to exit commander\n");

    while (1)
    {
        char  buf[100]="";
        char  command[100]="";
        char  param[100]="";
        char size[100]="";
        char flag[100]="";
        unsigned char  packet[UF_PACKET_LEN];
        int nCommand, nParam, nSize, nFlag;
        char *p;
        int s;

        printf("\r## ");
        fflush(stdin);
        gets(buf);
        p = strtok(buf, " ,");

        if(p!=NULL) {
            strcpy(command, p);
            if(strcmp(command,"exit")==0) {
                printf("\rcommander will be exit!\n");
                WaitKey();
                break;
            }
         } else {
            printf("\rWrong command! Try again!\n");
            continue;
         }

         p = strtok(NULL, " ,");
         if(p!=NULL)
            strcpy(param, p);
        else {
            printf("\rWrong command! Try again!\n");
            continue;
        }

        p = strtok(NULL, " ,");
        if(p!=NULL)
            strcpy(size, p);
        else
        {
            printf("\rWrong command! Try again!\n");
            continue;
        }

        p = strtok(NULL, " ,");
        if(p!=NULL)
            strcpy(flag, p);
        else {
           printf("\rWrong command! Try again!\n");
            continue; 
        }

        if(GetStrToInt(command) != 0)
        {
            nCommand = GetStrToInt(command);
        }
        else
        {
             nCommand = GetCommandParameterIndex(command);
        }
        nParam = GetStrToInt(param);
        nSize = GetStrToInt(size);
        nFlag = GetStrToInt(flag);

        UF_MakePacket(nCommand, nParam, nSize, nFlag, packet );
        SendPacket(packet);
        
        ClearBuffer();

//        s = GetTickCount();
        s = HAL_GetTick();
        
        while(1)
        {
            memset(packet,0,UF_PACKET_LEN);
            //RecievePacket(packet, UF_PACKET_LEN, 1000);
            HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
            if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SUCCESS &&  packet[UF_PACKET_END_CODE_POS]== UF_PACKET_END_CODE)
            {
                printf("\rERROR FLAG : 0x%02x\n", UF_GetPacketValue(UF_PACKET_FLAG, packet));
//                break;
            }

            //if(GetTickCount()-s > 10000)
            if(HAL_GetTick()-s > 10000)
                break;
        }
    
    }
}

int Connect(void)
{
    unsigned char send_buf[UF_PACKET_LEN]={0x40,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x0A};
    unsigned char recv_buf[UF_PACKET_LEN]="";
    int flag;
    int chksum;

    int ret;

    MX_USART1_UART_Init();
    
    //SendPacket(send_buf);
    //HAL_UART_Transmit(&huart1, (uint8_t *)&send_buf, 1, 1000);
    //RecievePacket(recv_buf, UF_PACKET_LEN, 3000);
    //ret = HAL_UART_Receive(&huart1, (uint8_t *)&recv_buf, 1, 1000);

    HAL_UART_Receive_IT(&huart1, recv_buf, UF_PACKET_LEN);
    HAL_UART_Transmit_IT(&huart1, send_buf, UF_PACKET_LEN);

    printf("\r[RX] ");
    for (int i = 0; i < UF_PACKET_LEN; i++)
    {
        HAL_Delay(4);   // The delay time may need to be changed.
        printf("%02X ", recv_buf[i]);
    }
    printf("\n");
    
    flag = UF_GetPacketValue(UF_PACKET_FLAG, recv_buf);
    chksum = UF_GetPacketValue(UF_PACKET_CHECKSUM, recv_buf);

    printf("\r[CONNECT] flag = %x\n", flag);
    printf("\r[CONNECT] chksum = %x\n", chksum);

    if(flag == UF_PROTO_RET_SUCCESS)
    {
      g_bConnectionStatus = 1;
      ReadSystemParameterSetting();
      printf("\r\n[ Port %d (%dbps)was connected ]\n\n", g_nPort, g_nBaudrate);
    }
    
    //Reconnect();
    WaitKey();

    return 1;
}

int Disconnect(void)
{
    if(g_bConnectionStatus ==  1)
    {
        HAL_UART_MspDeInit(&huart1);
        g_bConnectionStatus = 0;
        printf("\r\n[ Port was closed ]\n\n");
        WaitKey();
        return 1;
    }
    else
    {
        printf("\rPort was not connected. press any key!\n");
        WaitKey();
        return 0;
    }
}

void Reconnect(void)
{
    Disconnect();
    Connect();
    printf("Reconnected. press any key!\n");
    WaitKey();
}

void ReadSystemParameterSetting(void)
{
    int i, s;
    int param;
    int size;
    int flag;
    int result;
    BYTE packet[UF_PACKET_LEN];
    UF_InitSysParameter();

    i = 0;

    if (g_bConnectionStatus == 0) {
        printf("\rThe module was not connected. Please, Connect to module first.\n");
        WaitKey();
        return;
    }

    sysParameter = GetSysParamPointer();

    while( sysParameter[i].parameter != -1)
    {
        UF_MakePacket( UF_COM_SR , param, size, sysParameter[i].parameter, packet );
        SendPacket(packet);
        ClearBuffer();
        //RecievePacket(packet, UF_PACKET_LEN, 3000);
        HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
        HAL_Delay(300);
        
        result = UF_GetPacketValue(UF_PACKET_FLAG, packet);
        if(result == UF_PROTO_RET_SUCCESS) {
            sysParameter[i].value = UF_GetPacketValue(UF_PACKET_SIZE, packet);
        }

        i++;
    }

    i = 0;
    printf("\r======================================\n");
    while( sysParameter[i].parameter != -1)
    {
        printf("\r%20s : 0x%08x(%d)\n", sysParamList[i], sysParameter[i].value, sysParameter[i].value);
        i++;
    }
    printf("\r======================================\n");
}

void SystemParameterSetting(void)
{
    unsigned char strMenu[1]="";
    unsigned char c ;
    int nMenu;
    int bExit = 0;

    ReadSystemParameterSetting();

    do
    {
        printf("\rDo you want to change a parameter? [Y/N] : ");
        fflush(stdin);
        strMenu[0] = getchar();
        c = strMenu[0];
        if(c == 'y' || c == 'Y')
            break;
        else if(c == 'n' || c == 'N')
            return;
        else
        {
            printf("Please, try again!\n");
            WaitKey();
        }
    }while(bExit!=1);

    ChangeSystemParameterSetting();
    WaitKey();
}

void ChangeSystemParameterSetting(void)
{
    unsigned char strMenu[100]="";
    unsigned char paramID[20]="";
    unsigned char paramValue[10]="";
    BYTE packet[UF_PACKET_LEN];
    int nParamValue=0;
    int nParamID=0;
    int bExit = 0;
    char c;

    printf("Enter parameter name (see above table):");
    fflush(stdin);
    paramID[0] = getchar();
//    gets(paramID);
    nParamID = GetSystemParameterIndex(paramID);

    printf("Enter parameter value (integer or hex) :");
    fflush(stdin);
    strMenu[0] = getchar();
    nParamValue = atoi(strMenu);
    if(strtol(paramValue, NULL, 0) != 0L)
        nParamValue = strtol(paramValue, NULL, 0);
    else if(strtol(paramValue, NULL, 10) != 0L)
        nParamValue = strtol(paramValue, NULL, 10);
}

void CommunicationSetup(void)
{
    char strMenu[1];
    int nMenu=-1;
    int bExit = 0;
    int RcvStat;
    
    while(bExit!=1)
    {
        CommunicationSetupMenu();
        fflush(stdin);
        strMenu[0] = getchar();
        nMenu  = atoi(strMenu);
        switch(nMenu)
        {
            //Connect
            case 1:
                Connect();
                printf("\r Connect()\n");
                if(g_bConnectionStatus==1)
                    bExit=1;
                break;
            //Disconnect
            case 2:
                Disconnect();
                printf("\r Disconnect()\n");
                break;
            //Reconnect
            case 3:
                Reconnect();
                printf("\r Reconnect()\n");
                if(g_bConnectionStatus==1)
                    bExit=1;
                break;
            //Search module
            case 4:
    //            SearchModule();
                printf("\r SearchModule()\n");
                break;
            //Change Communication Setup
            case 5:
    //            ChangeCommunicationSetup();
                printf("\r ChangeCommunicationSetup()\n");
                break;
            //Exit
            case 6:
                bExit = 1;
                break;
            default:
                printf("Please, try again!\n");
                fflush(stdin);
                getchar();
                break;
        }
        nMenu = -1;
        ClearBuffer();
    }
}

void WaitKey(void)
{
    printf("\rpress any key!");
    fflush(stdin);
    getchar();
}

void BinToHex(unsigned char c , unsigned char *buf)
{
    TOHEX(c, buf);
}

void BinToHexPacket(unsigned char* inbuf, unsigned char *outbuf, int inbuf_size)
{
    int i;

    for(i=0; i<inbuf_size; i++)
    {
        BinToHex(inbuf[i], &outbuf[i*3]);
        outbuf[i*3+2] = 32;
    }

}

int GetCommandParameterIndex(char* paramName)
{
    int i=0;
    int result=-1;

    while(commandParamList[i].nCommand != -1)
    {
        if(strcmp(paramName, commandParamList[i].strCommand)==0)
        {
            result = commandParamList[i].nCommand;
            break;
        }
        i++;
    }
    return result;
}

int GetStrToInt(char* str)
{
    long int result=0;

    if(strtol(str, NULL, 0) != 0L)
        result = strtol(str, NULL, 0);
    else if(strtol(str, NULL, 10) != 0L)
        result = strtol(str, NULL, 10);

    return result;
}

void SendPacket(unsigned char* data)
{
    int i;
    unsigned char hexbuf[UF_PACKET_LEN*3];

    memset(hexbuf, 0, UF_PACKET_LEN);

    HAL_UART_Transmit_IT(&huart1, data, UF_PACKET_LEN);

    BinToHexPacket(data, hexbuf, UF_PACKET_LEN);

    printf("%8s : ", "[SEND]",UF_PACKET_LEN);

    for(i=0; i<UF_PACKET_LEN*3; i++)
    {
        printf("%c",(char)hexbuf[i]);
    }
    printf("\n");
}

void ClearBuffer(void)
{
#if 0
    huart1.gState = HAL_UART_STATE_RESET;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
#else
    memset(transmitBuffer, 0x0, 13);
    memset(receiveBuffer, 0x0, 13);
//    __HAL_UART_RESET_HANDLE_STATE(&huart1);
#endif
}

void Enroll(void)
{
    printf("\r[andro.yoon] %s\n", __func__);
    int result;
    UINT32 userID;
    int imageQaulity;
    int enroll_mode;
    int index;
    BYTE packet[UF_PACKET_LEN];
    int param = 0;
    int size = 0;
    int flag = UF_ENROLL_AUTO_ID;

    if(g_bConnectionStatus == 0)
    {
        printf("\rThe module was not connected. Please, Connect to module first.\n");
        WaitKey();
        return;
    }

	UF_MakePacket( UF_COM_ES , param, size, flag, packet );
	SendPacket(packet);

	//Wait(500);
	index = GetSystemParameterIndex("ENROLL_MODE");
	enroll_mode = sysParameter[index].value;

    if(enroll_mode == UF_ENROLL_TWO_TEMPLATES1 || enroll_mode == UF_ENROLL_TWO_TIMES1)
    {
        printf("\rPlace finger on the sensor\n");
        ClearBuffer();
        //RecievePacket(packet, UF_PACKET_LEN, 10000);
        HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
        if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SCAN_SUCCESS)
            printf("\r\n[ Scan Success ]\n\n");

        //Wait(500);
        ClearBuffer();
        //RecievePacket(packet, UF_PACKET_LEN, 10000);
        HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
        if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SUCCESS)
        {
            printf("\r\n[ Success ]\n\n");
            userID = UF_GetPacketValue(UF_PACKET_PARAM, packet);
            imageQaulity = UF_GetPacketValue(UF_PACKET_SIZE, packet);
            printf("\r\n[ Enrolled UserID : %d  Score : %d ]\n",userID, imageQaulity);
        }
    }
    if(enroll_mode == UF_ENROLL_TWO_TIMES2 || enroll_mode == UF_ENROLL_TWO_TEMPLATES2)
    {
        UF_MakePacket( UF_COM_ES , param, size, UF_ENROLL_CONTINUE, packet );

        SendPacket(packet);
    }

    printf("\rPlace finger on the sensor\n");
    ClearBuffer();
//    RecievePacket(packet, UF_PACKET_LEN, 10000);
    HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
    if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SCAN_SUCCESS)
        printf("\r\n[ Scan Success ]\n\n");

    //Wait(500);
    ClearBuffer();
//	RecievePacket(packet, UF_PACKET_LEN, 10000);
    HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
	if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SUCCESS)
	{
	    printf("\r\n[ Success ]\n\n");
	    userID = UF_GetPacketValue(UF_PACKET_PARAM, packet);
        imageQaulity = UF_GetPacketValue(UF_PACKET_SIZE, packet);
        printf("\r\n[ Enrolled UserID : %d  Score : %d ]\n",userID, imageQaulity);
	}


    ClearBuffer();

    WaitKey();
}

void Identify(void)
{
    printf("\r[andro.yoon] %s\n", __func__);
    int result;
    UINT32 userID;
    UINT32 subID;
    BYTE packet[UF_PACKET_LEN];
    int param = 0;
    int size = 0;
    int flag = 0;

    if(g_bConnectionStatus == 0)
    {
        printf("\rThe module was not connected. Please, Connect to module first.\n");
        WaitKey();
        return;
    }

    UF_MakePacket( UF_COM_IS , param, size, flag, packet );
    SendPacket(packet);

    printf("\r>>>Place finger on the sensor>>>\n");
    ClearBuffer();

    while (1) {
        HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
        if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SCAN_SUCCESS) {
            printf("\r\n[%s - Scan Success ]\n", __func__);
            printf("\r----------------------------------");
            break;
        } else if (UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_TIME_OUT) {
            printf("\r\n[%s - Time out ]\n", __func__);
            printf("\r----------------------------------");
            return;
        }
    }
    ClearBuffer();

    HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
    while (1) {
        HAL_Delay(50);

        fflush(stdin);
        result = UF_GetPacketValue(UF_PACKET_FLAG, packet);
        switch (result) {
            case UF_PROTO_RET_SUCCESS:
                printf("\r\n[%s - Success ]\n", __func__);
                userID = UF_GetPacketValue(UF_PACKET_PARAM, packet);
                subID = UF_GetPacketValue(UF_PACKET_SIZE, packet);
                printf("\r\n[ Identified UserID : %d(%d) ]\n",userID, subID);
                printf("\r----------------------------------");
                return;
            case UF_PROTO_RET_NOT_FOUND:
                printf("\r\n[%s - Not found ]\n", __func__);
                printf("\r----------------------------------");
                return;
            default:
                break;
        }
#if 0
        if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SUCCESS) {
            printf("\r\n[%s - Success ]\n", __func__);
            userID = UF_GetPacketValue(UF_PACKET_PARAM, packet);
            subID = UF_GetPacketValue(UF_PACKET_SIZE, packet);
            printf("\r\n[ Identified UserID : %d(%d) ]\n",userID, subID);
            break;
        } else {
            printf("\r\n[%s - Fail ]\n", __func__);
            return;
        }
#endif
    }
    ClearBuffer();

    WaitKey();
}

void Verify(void)
{
    printf("\r[andro.yoon] %s\n", __func__);
    int result;
    char strID[10];
    UINT32 userID=0;
    UINT32 subID=0;
    BYTE packet[UF_PACKET_LEN];
    int param = userID;
    int size = 0;
    int flag = 0;

    if(g_bConnectionStatus == 0)
    {
        printf("The module was not connected. Please, Connect to module first.\n");
        WaitKey();
        return;
    }

    printf("Please, enter the user ID : ");
    fflush(stdin);
    gets(strID);
    userID  = atoi(strID);

	UF_MakePacket( UF_COM_VS , userID, size, flag, packet );

	SendPacket(packet);

    printf("Place finger on the sensor\n");
    ClearBuffer();
    //RecievePacket(packet, UF_PACKET_LEN, 10000);
    HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
    
    if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SCAN_SUCCESS)
        printf("\n[ Scan Success ]\n\n");
    else
        return;

    ClearBuffer();
	//RecievePacket(packet, UF_PACKET_LEN, 10000);
    HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
	if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SUCCESS)
	{
	    printf("\n[ Success ]\n");
	    userID = UF_GetPacketValue(UF_PACKET_PARAM, packet);
        subID = UF_GetPacketValue(UF_PACKET_SIZE, packet);
        printf("\n[ Verified UserID : %d(%d) ]\n",userID, subID);
	}
	else
	{
	    printf("\n[ Verification was failed ]\n");
	}


    ClearBuffer();

    WaitKey();
}

void Delete_all(void)
{
    BYTE packet[UF_PACKET_LEN];
    int param = 0;
    int size = 0;
    int flag = 0;

    if(g_bConnectionStatus == 0) {
        printf("\rThe module was not connected. Please, Connect to module first.\n");
        WaitKey();
        return;
    }

    UF_MakePacket( UF_COM_DA , param, size, flag, packet );
    SendPacket(packet);

    while(1) {
        HAL_UART_Receive_IT(&huart1, packet, UF_PACKET_LEN);
        if(UF_GetPacketValue(UF_PACKET_FLAG, packet) == UF_PROTO_RET_SUCCESS) {
            printf("\r[%s] Delete All OK\n", __func__);
//            break;
            return;
        }
    }
    ClearBuffer();
    WaitKey();
}

int GetSystemParameterIndex(char* paramName)
{
    int i=0;
    int result=-1;
    if(sysParameter==NULL) {
        printf("\r[andro.yoon] sysParameter NULL\n");
        return -1;
    }

    while(sysParameter[i].parameter != -1)
    {
        if(strcmp(paramName, sysParamList[i])==0)
        {
            result = i;
            break;
        }
        i++;
    }
    return result;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
#if 0
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#if 0
/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}
#endif

/* USART1 init function */
static int MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
    return 1;
  }

  return 1;
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : USER_BTN_Pin PC_6_Pin PC_8_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin|PC_6_Pin|PC_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_Pin PB3 PB5 */
  GPIO_InitStruct.Pin = USER_LED_Pin|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_Pin|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
