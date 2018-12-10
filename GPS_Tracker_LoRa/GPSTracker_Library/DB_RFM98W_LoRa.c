/*
 * DB_RFM98W.c
 *
 *  Created on: 2 sie 2018
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "DB_RFM98W_LoRa.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>   // dla powf()

/********************** NOTES **********************************************

 #################### website: www.HopeRF.com ####################
#################### driver for RFM92/95/96/98 ####################
**lora_tx**
    byte str[21] = {'H','o','p','e','R','F',' ','R','F','M',' ','C','O','B','R','F','M','9','8','S'};

    void setup()
    {
     radio.Modulation     = LORA;
     radio.COB            = RFM98;
     radio.Frequency      = 434000;
     radio.OutputPower    = 17;             //17dBm OutputPower
     radio.PreambleLength = 16;             //16Byte preamble
     radio.FixedPktLength = false;          //explicit header mode for LoRa
     radio.PayloadLength  = 21;
     radio.CrcDisable     = true;

     radio.SFSel          = SF9;
     radio.BWSel          = BW125K;
     radio.CRSel          = CR4_5;

     radio.vInitialize();
     radio.vGoStandby();
    }

    void loop()
    {
     radio.bSendMessage(str, 21);
     delay(1000);
    }
**lora_rx**
    byte getstr[21];

    void setup()
    {
     radio.Modulation     = LORA;
     radio.COB            = RFM98;
     radio.Frequency      = 434000;
     radio.OutputPower    = 17;             //17dBm OutputPower
     radio.PreambleLength = 16;             //16Byte preamble
     radio.FixedPktLength = false;          //explicit header mode for LoRa
     radio.PayloadLength  = 21;
     radio.CrcDisable     = false;

     radio.SFSel          = SF9;
     radio.BWSel          = BW125K;
     radio.CRSel          = CR4_5;

     radio.vInitialize();
     radio.vGoRx();
     uart.vUartInit(9600, _8N1);
    }

    void loop()
    {
     if(radio.bGetMessage(getstr)!=0)
        {
        uart.vUartPutNByte(getstr, 21);
        uart.vUartNewLine();
        }
    }

*******************************************************************************/
/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* KONFIGURACJA -> TIMEOUT */
#define SPI_TIMEOUT     50  //  100

#ifdef FREERTOS
    #define _delay_ms(_ms)  osDelay(_ms)
#else
    #define _delay_ms(_ms)  HAL_Delay(_ms)
#endif

/**********************************************************
**RF69 Regsister define
**********************************************************/
//Common Regsister
#define     RegFifo             0x00
#define     RegOpMode           0x01
#define     RegFrMsb            0x06
#define     RegFrMid            0x07
#define     RegFrLsb            0x08
#define     RegPaConfig         0x09
#define     RegPaRamp           0x0A
#define     RegOcp              0x0B
#define     RegLna              0x0C

#define     RegDioMapping1      0x40
#define     RegDioMapping2      0x41
#define     RegVersion          0x42

//for 6/7/8
#define     RegPllHop           0x44
#define     RegTcxo             0x4B
#define     RegPaDac            0x4D
#define     RegFormerTemp       0x5B
#define     RegBitrateFrac      0x5D
#define     RegAgcRef           0x61
#define     RegAgcThresh1       0x62
#define     RegAgcThresh2       0x63
#define     RegAgcThresh3       0x64
#define     RegPll              0x70

//for 2/3
#define     RegAgcRef_2         0x43
#define     RegAgcThresh1_2     0x44
#define     RegAgcThresh2_2     0x45
#define     RegAgcThresh3_2     0x46
#define     RegPllHop_2         0x4B
#define     RegTcxo_2           0x58
#define     RegPaDac_2          0x5A
#define     RegPll_2            0x5C
#define     RegPllLowPn         0x5E
#define     RegFormerTemp_2     0x6C
#define     RegBitrateFrac_2    0x70

//ASK/FSK/GFSK Regsister
#define     RegBitrateMsb       0x02
#define     RegBitrateLsb       0x03
#define     RegFdevMsb          0x04
#define     RegFdevLsb          0x05

#define     RegRxConfig         0x0D
#define     RegRssiConfig       0x0E
#define     RegRssiCollision    0x0F
#define     RegRssiThresh       0x10
#define     RegRssiValue        0x11
#define     RegRxBw             0x12
#define     RegAfcBw            0x13
#define     RegOokPeak          0x14
#define     RegOokFix           0x15
#define     RegOokAvg           0x16
#define     RegAfcFei           0x1A
#define     RegAfcMsb           0x1B
#define     RegAfcLsb           0x1C
#define     RegFeiMsb           0x1D
#define     RegFeiLsb           0x1E
#define     RegPreambleDetect   0x1F
#define     RegRxTimeout1       0x20
#define     RegRxTimeout2       0x21
#define     RegRxTimeout3       0x22
#define     RegRxDelay          0x23
#define     RegOsc              0x24
#define     RegPreambleMsb      0x25
#define     RegPreambleLsb      0x26
#define     RegSyncConfig       0x27
#define     RegSyncValue1       0x28
#define     RegSyncValue2       0x29
#define     RegSyncValue3       0x2A
#define     RegSyncValue4       0x2B
#define     RegSyncValue5       0x2C
#define     RegSyncValue6       0x2D
#define     RegSyncValue7       0x2E
#define     RegSyncValue8       0x2F
#define     RegPacketConfig1    0x30
#define     RegPacketConfig2    0x31
#define     RegPayloadLength    0x32
#define     RegNodeAdrs         0x33
#define     RegBroadcastAdrs    0x34
#define     RegFifoThresh       0x35
#define     RegSeqConfig1       0x36
#define     RegSeqConfig2       0x37
#define     RegTimerResol       0x38
#define     RegTimer1Coef       0x39
#define     RegTimer2Coef       0x3A
#define     RegImageCal         0x3B
#define     RegTemp             0x3C
#define     RegLowBat           0x3D
#define     RegIrqFlags1        0x3E
#define     RegIrqFlags2        0x3F


//LoRa Regsister
#define     RegFifoAddrPtr              0x0D
#define     RegFifoTxBaseAddr           0x0E
#define     RegFifoRxBaseAddr           0x0F
#define     RegFifoRxCurrentAddr        0x10
#define     RegIrqFlagsMask             0x11
#define     RegIrqFlags                 0x12
#define     RegRxNbBytes                0x13
#define     RegRxHeaderCntValueMsb      0x14
#define     RegRxHeaderCntValueLsb      0x15
#define     RegRxPacketCntValueMsb      0x16
#define     RegRxPacketCntValueLsb      0x17
#define     RegModemStat                0x18
#define     RegPktSnrValue              0x19
#define     RegPktRssiValue             0x1A
#define     RegRssiValue_LR             0x1B
#define     RegHopChannel               0x1C
#define     RegModemConfig1             0x1D
#define     RegModemConfig2             0x1E
#define     RegSymbTimeoutLsb           0x1F
#define     RegPreambleMsb_LR           0x20
#define     RegPreambleLsb_LR           0x21
#define     RegPayloadLength_LR         0x22
#define     RegMaxPayloadLength         0x23
#define     RegHopPeriod                0x24
#define     RegFifoRxByteAddr           0x25
#define     RegModemConfig3             0x26        // only for 6/7/8



/**********************************************************
**RF69 mode status
**********************************************************/
#define     RADIO_SLEEP         (0x00)
#define     RADIO_STANDBY       (0x01)
#define     RADIO_TX            (0x03)
#define     RADIO_RX            (0x05)

#define     FskMode             (0<<5)
#define     OokMode             (1<<5)

#define     Shaping             2

#define     MODE_MASK           0xF8

#define     MOUDLE_MASK_2       0x87            //for RFM92/93
#define     MOUDLE_MASK_1       0x9F            //for RFM95/96/97/98

#define     AFC_ON              (1<<4)
#define     AGC_ON              (1<<3)
#define     RX_TRIGGER          0x06

#define     PREAMBLE_DECT_ON    (1<<7)
#define     PREAMBLE_DECT_1BYTE (0<<5)
#define     PREAMBLE_DECT_2BYTE (1<<5)
#define     PREAMBLE_DECT_3BYTE (2<<5)

#define     AUTO_RST_RX_OFF     (0<<6)
#define     AUTO_RST_RX_ON      (1<<6)
#define     AUTO_RST_RX_ONwPLL  (2<<6)
#define     SYNC_ON             (1<<4)

//for PacketConfig
#define     VariablePacket      (1<<7)
#define     DcFree_NRZ          (0<<5)
#define     DcFree_MANCHESTER   (1<<5)
#define     DcFree_WHITENING    (2<<5)
#define     CrcOn               (1<<4)
#define     CrcDisAutoClear     (1<<3)
#define     AddrFilter_NONE     (0<<1)
#define     AddrFilter_NODE     (1<<1)
#define     AddrFilter_ALL      (2<<1)
#define     CrcCalc_CCITT       0x00
#define     CrcCalc_IBM         0x01

#define     PacketMode          (1<<6)
#define     ContinuousMode      (0<<6)

//for LoRa
#define     AllIrqMask              0xFF
#define     RxTimeoutMask           (1<<7)
#define     RxDoneMask              (1<<6)
#define     PayloadCrcErrorMask     (1<<5)
#define     ValidHeaderMask         (1<<4)
#define     TxDoneMask              (1<<3)
#define     CadDoneMask             (1<<2)
#define     FhssChangeChannelMask   (1<<1)
#define     CadDetectedMask         (1<<0)


/* Types definition (definicje typow) --------------------------------------------------------------------------------*/
/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/
/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_RFM98W;
static GPIO_TypeDef* SPI_CS_GPIOx;
static uint16_t SPI_CS_GPIO_Pin;
static GPIO_TypeDef* RESET_GPIOx;
static uint16_t RESET_GPIO_Pin;

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
static void SPI_CS_pin(GPIO_PinState _PinState);
#if 0
static uint8_t WriteRegister(uint8_t addr, uint8_t value);
static uint8_t ReadRegister(uint8_t addr);
#endif

inline static void SetPOR(void);
inline static void ClrPOR(void);

static uint8_t DIO0_H(RFM98_LoRa_data_t *_data);

static byte bSpiTransfer(byte dat);
static void vSpiWrite(word dat);
static byte bSpiRead(byte addr);
static void vSpiBurstWrite(byte addr, byte ptr[], byte length);
static void vSpiBurstRead(byte addr, byte ptr[], byte length);

static void RFM98W_LoRa_vReset(RFM98_LoRa_data_t *_data);
static byte RFM98W_LoRa_bSelectBandwidth(byte rx_bw);
static byte RFM98W_LoRa_bSelectRamping(lword symbol);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

uint8_t RFM98W_LoRa_init(RFM98_LoRa_data_t *_data,
                    SPI_HandleTypeDef *_hspi_RFM98W,
                    GPIO_TypeDef* _SPI_CS_GPIOx,
                    uint16_t _SPI_CS_GPIO_Pin,
                    GPIO_TypeDef* _RESET_GPIOx,
                    uint16_t _RESET_GPIO_Pin)
{
    uint8_t status = 0;
    hspi_RFM98W = _hspi_RFM98W;
    SPI_CS_GPIOx = _SPI_CS_GPIOx;
    SPI_CS_GPIO_Pin = _SPI_CS_GPIO_Pin;

    RESET_GPIOx = _RESET_GPIOx;
    RESET_GPIO_Pin = _RESET_GPIO_Pin;

    _data->FrequencyValue.Freq = (_data->Frequency<<11)/125;        //calc frequency
    _data->BitRateValue  = (_data->SymbolTime<<5)/1000;             //calc bitrate
    _data->DevationValue = (_data->Devation<<11)/125;               //calc deviation
    _data->BandWidthValue = RFM98W_LoRa_bSelectBandwidth(_data->BandWidth);

    switch(_data->SFSel)
    {
        case SF6:  _data->SFValue = 6; break;
        case SF7:  _data->SFValue = 7; break;
        case SF8:  _data->SFValue = 8; break;
        case SF10: _data->SFValue = 10;break;
        case SF11: _data->SFValue = 11;break;
        case SF12: _data->SFValue = 12;break;
        case SF9:
        default:   _data->SFValue = 9; break;
    }
    switch(_data->BWSel)
    {
        case BW62K:  _data->BWValue = 6; break;             //for RFM95/96/97/98
        case BW250K: _data->BWValue = 8; break;
        case BW500K: _data->BWValue = 9; break;
        case BW125K:
        default:     _data->BWValue = 7; break;
    }
    switch(_data->CRSel)
    {
        default:
        case CR4_5: _data->CRValue = 1; break;
        case CR4_6: _data->CRValue = 2; break;
        case CR4_7: _data->CRValue = 3; break;
        case CR4_8: _data->CRValue = 4; break;
    }

    if((_data->SFValue-4) >= _data->BWValue)
        _data->RsOptimize = true;
    else
        _data->RsOptimize = false;

    RFM98W_LoRa_vConfig(_data);
    RFM98W_LoRa_vGoStandby();

    status = true;

    return status;
}


/**********************************************************
**Name:     vConfig
**Function: config rfm9x
**Input:    none
**Output:   none
**********************************************************/
void RFM98W_LoRa_vConfig(RFM98_LoRa_data_t *_data)
{
    byte i, j;
    byte sync;

    RFM98W_LoRa_vReset(_data);
    RFM98W_LoRa_vGoStandby();

    //Frequency
    vSpiWrite(((word)RegFrMsb<<8)+_data->FrequencyValue.freq.FreqH);
    vSpiWrite(((word)RegFrMid<<8)+_data->FrequencyValue.freq.FreqM);
    vSpiWrite(((word)RegFrLsb<<8)+_data->FrequencyValue.freq.FreqL);

    //PA Config
    i = bSpiRead(RegPaConfig);
    i &= 0x70;
    if(_data->OutputPower>=20)
    { i |= 0x0F; j = 0x87; }
    else if(_data->OutputPower>17)
    { i |= (_data->OutputPower-3-2); j = 0x87; }
    else if(_data->OutputPower>=2)
    { i |= (_data->OutputPower-2); j = 0x84; }
    else
    { i |= 0; j = 0x84; }
    vSpiWrite(((word)RegPaConfig<<8)+0x80+i);   //PA_BOOST
    switch(_data->COB)
    {
        case RFM92:
        case RFM93: vSpiWrite(((word)RegPaDac_2<<8)+j); break;
        default:    vSpiWrite(((word)RegPaDac<<8)+j); break;
    }
    j = bSpiRead(RegPaRamp);
    j&= 0x0F;
    j|= RFM98W_LoRa_bSelectRamping(_data->SymbolTime);
    vSpiWrite(((word)RegPaRamp<<8)+j);

    //Ocp
    vSpiWrite(((word)RegOcp<<8)+0x0F);          //Disable Ocp

    //LNA
    //vSpiWrite(((word)RegLna<<8)+0x20);        //High & LNA Enable



    // Mode
    i = bSpiRead(RegOpMode);
    j = bSpiRead(RegPaRamp);        //for RFM95/96/97/98
    if(_data->Modulation==LORA)
    {
        i &= 0x87;
        switch(_data->COB)
        {
            case RFM96:
            case RFM98: i |= 0x08; break;
            default: break;
        }
        vSpiWrite(((word)RegOpMode<<8)+i);

        i = bSpiRead(0x31);         //SetNbTrigPeaks
        i &= 0xF8;
        if(_data->SFSel==SF6)
        {
            i |= 0x05;
            vSpiWrite(0x3100+i);
            vSpiWrite(0x3700+0x0C);
            _data->FixedPktLength = true;           //SF6 must be ImplicitHeaderMode
        }
        else
        {
            i |= 0x03;
            vSpiWrite(0x3100+i);
        }

        byte tmp;
        switch(_data->COB)
        {
            case RFM92:
            case RFM93:
                if(_data->BWValue>6)
                    tmp = _data->BWValue - 7;
                else
                    tmp = 0;
                tmp <<= 6;                  //BandWidth
                tmp |= (_data->CRValue<<3);
                if(_data->FixedPktLength)           //ImplicitHeader
                    tmp |= 0x04;
                if(!_data->CrcDisable)              //
                    tmp |= 0x02;
                if(_data->RsOptimize)               //mandated for when the symbol length exceeds 16ms
                    tmp |= 0x01;
                vSpiWrite(((word)RegModemConfig1<<8)+tmp);
                tmp = (_data->SFValue<<4);          //SF rate
                tmp |= (0x04+0x03);         //AGC ON & Max timeout
                vSpiWrite(((word)RegModemConfig2<<8)+tmp);
            break;
            case RFM95:
            case RFM97:
            case RFM96:
            case RFM98:
            default:
                tmp = (_data->BWValue<<4) + (_data->CRValue<<1);
                if(_data->FixedPktLength)
                    tmp |= 0x01;
                vSpiWrite(((word)RegModemConfig1<<8)+tmp);
                tmp = (_data->SFValue<<4);
                if(!_data->CrcDisable)
                    tmp |= 0x04;
                tmp += 0x03;
                vSpiWrite(((word)RegModemConfig2<<8)+tmp);
                tmp = 0x04;             //AGC ON
                if(_data->RsOptimize)           //mandated for when the symbol length exceeds 16ms
                    tmp |= 0x08;
                vSpiWrite(((word)RegModemConfig3<<8)+tmp);
            break;
        }
        vSpiWrite(((word)RegSymbTimeoutLsb<<8)+0xFF);   //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
        vSpiWrite(((word)RegPreambleMsb_LR<<8)+(byte)(_data->PreambleLength>>8));
        vSpiWrite(((word)RegPreambleLsb_LR<<8)+(byte)_data->PreambleLength);

        vSpiWrite(((word)RegDioMapping2<<8)+0x40);                      //RegDioMapping2    DIO5=00(ModeReady), DIO4=01(PllLock)
    }
    else
    {
        switch(_data->COB)
        {
            case RFM92:
            case RFM93:
                i &= MOUDLE_MASK_2;
                switch(_data->Modulation)
                {
                    case OOK:  i |= OokMode+(Shaping<<3); break;
                    case GFSK: i |= FskMode+(Shaping<<3); break;
                    case FSK:
                    default:   i |= FskMode; break;
                }
            break;
            default:
                i &= MOUDLE_MASK_1;
                j &= 0x9F;
                switch(_data->Modulation)
                {
                    case OOK:  i |= OokMode; j |= (Shaping<<5); break;
                    case GFSK: i |= FskMode; j |= (Shaping<<5); break;
                    case FSK:
                    default:   i |= FskMode; break;
                }
            break;
        }
        vSpiWrite(((word)RegOpMode<<8)+i);
        vSpiWrite(((word)RegPaRamp<<8)+j);

        //BitRate
        vSpiWrite(((word)RegBitrateMsb<<8)+(byte)(_data->BitRateValue>>8));
        vSpiWrite(((word)RegBitrateLsb<<8)+(byte)_data->BitRateValue);

        //Devation
        vSpiWrite(((word)RegFdevMsb<<8)+(((byte)(_data->DevationValue>>8))&0x3F));
        vSpiWrite(((word)RegFdevLsb<<8)+(byte)(_data->DevationValue&0xFF));

        //RxConfig
        vSpiWrite(((word)RegRxConfig<<8)+AGC_ON+RX_TRIGGER);

        //RxBw
        vSpiWrite(((word)RegRxBw<<8)+_data->BandWidthValue);

        //OOK
        vSpiWrite(((word)RegOokPeak<<8)+0x20+(0x02<<3)+0x00);

        //PreambleDetect
        vSpiWrite(((word)RegPreambleDetect<<8)+PREAMBLE_DECT_ON+PREAMBLE_DECT_3BYTE);
        vSpiWrite(((word)RegPreambleMsb<<8)+(byte)(_data->PreambleLength>>8));
        vSpiWrite(((word)RegPreambleLsb<<8)+(byte)_data->PreambleLength);

        //Osc
        vSpiWrite(((word)RegOsc<<8)+0x07);          //Close OscClk Output

        //SyncConfig
        if(_data->SyncLength==0)
            sync = 0;
        else
            sync = _data->SyncLength-1;
        vSpiWrite(((word)RegSyncConfig<<8)+AUTO_RST_RX_ONwPLL+SYNC_ON+(sync&0x07));
        for(i=0;i<8;i++)                                //SyncWordSetting
            vSpiWrite(((word)(RegSyncValue1+i)<<8)+_data->SyncWord[i]);

        i = DcFree_NRZ + AddrFilter_NONE + CrcCalc_CCITT;
        if(!_data->FixedPktLength)
            i += VariablePacket;
        if(!_data->CrcDisable)
            i += CrcOn;
        vSpiWrite(((word)RegPacketConfig1<<8)+i);
        vSpiWrite(((word)RegPacketConfig2<<8)+PacketMode);

        if(_data->FixedPktLength)                               //Set Packet length
            vSpiWrite(((word)RegPayloadLength<<8)+_data->PayloadLength);
        else
            vSpiWrite(((word)RegPayloadLength<<8)+0xFF);

        vSpiWrite(((word)RegFifoThresh<<8)+0x01);
        vSpiWrite(((word)RegDioMapping2<<8)+0x61);  //DIO4 PllLock / DIO5 Data / PreambleDetect
    }
}

/**********************************************************
**Name:     vGoRx
**Function: set rf9x to receive mode
**Input:    none
**Output:   none
**********************************************************/
void RFM98W_LoRa_vGoRx(RFM98_LoRa_data_t *_data)
{
    byte tmp;
    if(_data->Modulation==LORA)
    {
        if(_data->FixedPktLength)                                   //Set Packet length
            vSpiWrite(((word)RegPayloadLength_LR<<8)+_data->PayloadLength);
        else
            vSpiWrite(((word)RegPayloadLength_LR<<8)+0xFF);

        vSpiWrite(((word)RegDioMapping1<<8)+0x22);      //DIO0 RxDone / DIO1 CadDetected / DIO2 FhssChangeChannel /DIO3 PayloadCrcError
        vSpiWrite(((word)RegIrqFlags<<8)+AllIrqMask);   //Clear All Interrupt
        vSpiWrite(((word)RegIrqFlagsMask<<8)+(AllIrqMask&(~(RxDoneMask|RxTimeoutMask))));   //just enable RxDone & Timeout

        tmp = bSpiRead(RegFifoRxBaseAddr);              //Read RxBaseAddr
        vSpiWrite(((word)RegFifoAddrPtr<<8)+tmp);       //RxBaseAddr -> FiFoAddrPtr¡¡
    }
    else
    {
        if(_data->CrcDisable)
            vSpiWrite(((word)RegDioMapping1<<8)+0x00);  //DIO0 PayloadReady / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty
        else
            vSpiWrite(((word)RegDioMapping1<<8)+0x40);  //DIO0 CrcOk  / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty
    }

    tmp = bSpiRead(RegOpMode);
    tmp&= MODE_MASK;
    tmp |= RADIO_RX;
    vSpiWrite(((word)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     vGoStandby
**Function: set rf9x to standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM98W_LoRa_vGoStandby(void)
{
    byte tmp;
    tmp = bSpiRead(RegOpMode);
    tmp&= MODE_MASK;
    tmp |= RADIO_STANDBY;
    vSpiWrite(((word)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     vGoSleep
**Function: set rf9x to sleep mode
**Input:    none
**Output:   none
**********************************************************/
void RFM98W_LoRa_vGoSleep(void)
{
    byte tmp;
    tmp = bSpiRead(RegOpMode);
    tmp&= MODE_MASK;
    tmp |= RADIO_SLEEP;
    vSpiWrite(((word)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     bSendMessage
**Function: set rf9x to sleep mode
**Input:    msg------for which message to send
            length---message length
**Output:   true-----send ok
            false----send error/over time
**********************************************************/
uint8_t RFM98W_LoRa_bSendMessage(RFM98_LoRa_data_t *_data, byte msg[], byte length)
{
    byte tmp;
    lword overtime;
    word bittime;

    if(_data->Modulation==LORA)
    {
        vSpiWrite(((word)RegPayloadLength_LR<<8)+length);
        vSpiWrite(((word)RegDioMapping1<<8)+0x62);      //DIO0 TxDone / DIO1 CadDetected / DIO2 FhssChangeChannel /DIO3 PayloadCrcError
        vSpiWrite(((word)RegIrqFlags<<8)+AllIrqMask);   //Clear All Interrupt
        vSpiWrite(((word)RegIrqFlagsMask<<8)+(AllIrqMask&(~TxDoneMask)));   //just enable TxDone

        tmp = bSpiRead(RegFifoTxBaseAddr);              //Read TxBaseAddr
        vSpiWrite(((word)RegFifoAddrPtr<<8)+tmp);       //RxBaseAddr -> FiFoAddrPtr¡¡

        vSpiBurstWrite(RegFifo, msg, length);
    }
    else
    {
        vSpiWrite(((word)RegDioMapping1<<8)+0x00);  //DIO0 PacketSend  / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty

        if(!_data->FixedPktLength)
            vSpiWrite(((word)RegFifo<<8)+length);
        vSpiBurstWrite(RegFifo, msg, length);
    }

    tmp = bSpiRead(RegOpMode);
    tmp&= MODE_MASK;
    tmp |= RADIO_TX;
    vSpiWrite(((word)RegOpMode<<8)+tmp);

    if(_data->Modulation==LORA)
    {
        overtime = _data->PreambleLength + 5 + 8;   //unit: byte
        bittime  = (length<<3) + 28 - (_data->SFValue<<2);
        if(!_data->CrcDisable)
            bittime += 16;
        if(_data->FixedPktLength)
            bittime -= 20;
        switch(_data->CRSel)
        {
            default:
            case CR4_5: bittime *= 5; break;
            case CR4_6: bittime *= 6; break;
            case CR4_7: bittime *= 7; break;
            case CR4_8: bittime *= 8; break;
        }
        bittime >>= 2;
        if(_data->RsOptimize)
            bittime /= 10;
        else
            bittime /= 12;
        overtime += bittime;            //unit: byte,  total = Payload + Preamble + Header

        if(_data->SFValue >= _data->BWValue)            //unit: ms
            overtime <<= (_data->SFValue - _data->BWValue);
        else
            overtime >>= (_data->BWValue - _data->SFValue);
        _delay_ms(overtime);            //
#ifdef DIO0_WORKING
        for(bittime=0;bittime<5000;bittime++)   //about 500ms for overtime
        {
            if(DIO0_H(_data))
                break;
            _delay_ms(1); // _delay_us(100);
        }
#endif
        vSpiWrite(((word)RegIrqFlags<<8)+AllIrqMask);   //Clear All Interrupt
        RFM98W_LoRa_vGoStandby();
        if(bittime>=5000)
            return(false);
        else
            return(true);
    }
    else
    {
        //
        bittime  = _data->SymbolTime/1000;      //unit: us
        overtime = _data->SyncLength+_data->PreambleLength+length;
        if(!_data->FixedPktLength)              //SyncWord & PktLength & 2ByteCRC
            overtime += 1;
        if(!_data->CrcDisable)
            overtime += 2;
        overtime<<=3;                       //8bit == 1byte
        overtime*= bittime;
        overtime/= 1000;                    //unit: ms
        if(overtime==0)
            overtime = 1;
        _delay_ms(overtime);                //
#ifdef DIO0_WORKING
        for(tmp=0;tmp<100;tmp++)            //about 10ms for overtime
        {
            if(DIO0_H(_data))
                break;
            _delay_ms(1);   // _delay_us(100);
        }
#endif
        RFM98W_LoRa_vGoStandby();
        if(tmp>=100)
            return(false);
        else
            return(true);
    }
}

/**********************************************************
**Name:     bGetMessage
**Function: check receive packet
**Input:    msg------for which message to read
**Output:   packet length
            0--------have no packet
**********************************************************/
byte RFM98W_LoRa_bGetMessage(RFM98_LoRa_data_t *_data, byte msg[])
{
    byte length;
    if(DIO0_H(_data))               //Receive CrcOk or PayloadReady
    {
        if(_data->Modulation==LORA)
        {
            byte addr;
            bSpiRead(RegIrqFlags);                      //Read Interrupt Flag for do something

            bSpiRead(RegPktSnrValue);
            bSpiRead(RegRssiValue_LR);
            bSpiRead(RegPktRssiValue);

            addr = bSpiRead(RegFifoRxCurrentAddr);
            vSpiWrite(((word)RegFifoAddrPtr<<8)+addr);
            length = bSpiRead(RegRxNbBytes);
            vSpiBurstRead(RegFifo, msg, length);
            vSpiWrite(((word)RegIrqFlags<<8)+AllIrqMask);   //Clear All Interrupt
        }
        else
        {
            if(_data->FixedPktLength)
                length = _data->PayloadLength;
            else
                length = bSpiRead(RegFifo);
            vSpiBurstRead(RegFifo, msg, length);
        }
        RFM98W_LoRa_vGoStandby();
        RFM98W_LoRa_vGoRx(_data);
        return(length);
    }
    return(0);
}


/**********************************************************
**Name:     iGetRSSI
**Function:
**Input:    void
**Output:   RSSI value.
**********************************************************/
int16_t RFM98W_LoRa_iGetRSSI(void)
{
    uint8_t rssi = RFM98W_LoRa_ReadRegister(RegRssiValue_LR);
    return (int16_t)rssi - 137;
}

int16_t RFM98W_LoRa_iGetRSSIPacket(void)
{
    uint8_t rssi = RFM98W_LoRa_ReadRegister(RegPktRssiValue);
    return (int16_t)rssi - 137;
//    return (RFM98W_LoRa_ReadRegister(RegPktRssiValue) - (_frequency < 868E6 ? 164 : 157));
}

/**
 * @brief Measure dinstance from RSSI value.
 * @param rssi: RSSI value.
 * @param txPower:
 * @retval distance [m].
 */
int32_t RSSI_calculateDistance(int16_t rssi, int16_t txPower)
{
#if 0   //  https://gist.github.com/eklimcz/446b56c0cb9cfe61d575
//  int8_t txPower = -59; //hard coded power value. Usually ranges between -59 to -65
    if (rssi == 0) {
        return -1;  //  -1.0;
    }

    float ratio = (float)rssi*1.0/txPower;
    if (ratio < 1.0)
    {
        return (int32_t)powf(ratio,10);
    }
    else
    {
        int32_t distance =  (0.89976)*powf(ratio,7.7095) + 0.111;
        return distance;
    }
#elif 0 //  https://www.quora.com/How-do-I-calculate-distance-in-meters-km-yards-from-rssi-values-in-dBm-of-BLE-in-android
    /*
     * RSSI = TxPower - 10 * n * lg(d)
     * n = 2 (in free space)
     *
     * d = 10 ^ ((TxPower - RSSI) / (10 * n))
     */
    return (int32_t)powf(10.0f, ((float) txPower - rssi) / (10.0f * 2.0f));
#elif 0 //  https://docs.google.com/spreadsheets/d/1ymREowDj40tYuA5CXd4IfC4WYPXxlx5hq1x8tQcWWCI/edit#gid=0
    //  Step 1: Caclulate Ratio
    float ratio = (float)rssi*1.0/txPower;
    //...
#elif 1
    const uint8_t N = 2;    //  propagation constant
    const uint8_t d0 = 1;   //  reference distance: 1 [m],
    //  RSSI0 – RSSI read-out at reference distance d0, in [dBm].
    //  RSSI0 -> txPower
    // d – distance between receiver and transmitter (beacon), in [m],
    int32_t d = d0 * powf( 10.0f, (float)(txPower - rssi)/(10.0f*N));
    return d;
#endif
}

//********************************* PRIVATE FUNCTION ****************************************************************

/**********************************************************
**Name:     vReset
**Function: hardware reset rf69 chipset
**Input:    none
**Output:   none
**********************************************************/
static void RFM98W_LoRa_vReset(RFM98_LoRa_data_t *_data)
{
    byte tmp;
//  POROut();
    switch(_data->COB)
    {
        case RFM92:                     //High Reset; Normal for Low
        case RFM93:
            ClrPOR();
            _delay_ms(1); // _delay_us(200);                //at least 100us for reset
            SetPOR();
            break;
        case RFM95:
        case RFM96:
        case RFM97:
        case RFM98:
        default:
            ClrPOR();   //  SetPOR();   //  Blad w plikach bilbiotecznych "HopeRFLib"
            _delay_ms(1); // _delay_us(200);                //at least 100us for reset
            SetPOR();   //  ClrPOR();
            break;
    }
//  PORIn();                            //set POR for free
    _delay_ms(6);                       //wait for ready
//  ClrPOR();                           //note: help for LowReset

    tmp = bSpiRead(RegOpMode);
    tmp&= MODE_MASK;
    tmp |= RADIO_SLEEP;
    vSpiWrite(((word)RegOpMode<<8)+tmp);

    tmp &= 0x7F;
    if(_data->Modulation==LORA)
    tmp |= 0x80;
    vSpiWrite(((word)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     bSelectBandwidth
**Function:
**Input:    BandWidth
**Output:   BandWidthValue
**********************************************************/
static byte RFM98W_LoRa_bSelectBandwidth(byte rx_bw)
{
    if(rx_bw<=10)
        return 0x15;                        //10.4KHz   Min
    else if(rx_bw<13)
        return 0x0D;                        //12.5KHz
    else if(rx_bw<16)
        return 0x05;                        //15.6KHz
    else if(rx_bw<21)
        return 0x14;                        //20.8KHz
    else if(rx_bw<=25)
        return 0x0C;                        //25.0KHz
    else if(rx_bw<32)
        return 0x04;                        //31.3KHz
    else if(rx_bw<42)
        return 0x13;                        //41.7KHz
    else if(rx_bw<=50)
        return 0x0B;                        //50.0KHz
    else if(rx_bw<63)
        return 0x03;                        //62.5KHz
    else if(rx_bw<84)
        return 0x12;                        //83.3KHz
    else if(rx_bw<=100)
        return 0x0A;                        //100KHz
    else if(rx_bw<=125)
        return 0x02;                        //125KHz
    else if(rx_bw<167)
        return 0x11;                        //167KHz
    else if(rx_bw<=200)
        return 0x09;                        //200KHz
    else if(rx_bw<=250)
        return 0x01;                        //250KHz
    else if(rx_bw<=333)
        return 0x10;                        //333KHz
    else if(rx_bw<=400)
        return 0x08;                        //400KHz
    else
        return 0x00;                        //500KHz Max
}

/**********************************************************
**Name:     bSelectRamping
**Function:
**Input:    symbol time
**Output:   ramping value
**********************************************************/
static byte RFM98W_LoRa_bSelectRamping(lword symbol)
{
    lword SymbolRate;

    SymbolRate = symbol/1000;           //ns->us
    SymbolRate = SymbolRate/4;          // 1/4 ramping

    if(SymbolRate<=10)
        return 0x0F;                    //10us
    else if(SymbolRate<=12)
        return 0x0E;                    //12us
    else if(SymbolRate<=15)
        return 0x0D;                    //15us
    else if(SymbolRate<=20)
        return 0x0C;                    //20us
    else if(SymbolRate<=25)
        return 0x0B;                    //25us
    else if(SymbolRate<=31)
        return 0x0A;                    //31us
    else if(SymbolRate<=40)
        return 0x09;                    //40us
    else if(SymbolRate<=50)
        return 0x08;                    //50us
    else if(SymbolRate<=62)
        return 0x07;                    //62us
    else if(SymbolRate<=100)
        return 0x06;                    //100us
    else if(SymbolRate<=125)
        return 0x05;                    //125us
    else if(SymbolRate<=250)
        return 0x04;                    //250us
    else if(SymbolRate<=500)
        return 0x03;                    //500us
    else if(SymbolRate<=1000)
        return 0x02;                    //1000us
    else if(SymbolRate<=2000)
        return 0x01;                    //2000us
    else
        return 0x00;
}

//**********************************************************************************************************


uint8_t RFM98W_LoRa_checkInterrupt(RFM98_LoRa_data_t *_data)
{
//    return digitalRead(this->DIO0_PIN);
    if( _data->interruptFlag.bit.DIO0 )
    {
        _data->interruptFlag.bit.DIO0 = false;
        return true;
    }
    return false;
}


/**
  * @brief  Read byte from register.
  * @param  addr: address
  * @retval register value.
  */
uint8_t RFM98W_LoRa_ReadRegister(uint8_t addr)
{
    return bSpiRead(addr);
}

/**********************************************************
**Name:     bSpiTransfer
**Func:     Transfer One Byte by SPI
**Input:
**Output:
**********************************************************/
static byte bSpiTransfer(byte dat)
{
    uint8_t dataR = 0;
#ifdef HAL_SPI_MODULE_ENABLED
    HAL_SPI_TransmitReceive(hspi_RFM98W, &dat, &dataR, 1, SPI_TIMEOUT);
#endif /* HAL_SPI_MODULE_ENABLED */
    return dataR;
}

/**********************************************************
**Name:     vSpiWrite
**Func:     SPI Write One word
**Input:    Write word
**Output:   none
**********************************************************/
static void vSpiWrite(word dat)
{
    SPI_CS_pin(GPIO_PIN_RESET);
    bSpiTransfer((byte)(dat>>8)|0x80);
    bSpiTransfer((byte)dat);
    SPI_CS_pin(GPIO_PIN_SET);
}

/**********************************************************
**Name:     bSpiRead
**Func:     SPI Read One byte
**Input:    readout addresss
**Output:   readout byte
**********************************************************/
static byte bSpiRead(byte addr)
{
    byte tmp;
    SPI_CS_pin(GPIO_PIN_RESET);
    bSpiTransfer(addr);
    tmp = bSpiTransfer(0xFF);
    SPI_CS_pin(GPIO_PIN_SET);
    return(tmp);
}

/**********************************************************
**Name:     vSpiBurstWirte
**Func:     burst wirte N byte
**Input:    array length & start address & head pointer
**Output:   none
**********************************************************/
static void vSpiBurstWrite(byte addr, byte ptr[], byte length)
{
    byte i;
    SPI_CS_pin(GPIO_PIN_RESET);
    bSpiTransfer(addr|0x80);
    for(i=0; i<length; i++)
        bSpiTransfer(ptr[i]);
    SPI_CS_pin(GPIO_PIN_SET);
}

/**********************************************************
**Name:     vSpiBurstRead
**Func:     burst read N byte
**Input:    array length & start address & head pointer
**Output:   none
**********************************************************/
static void vSpiBurstRead(byte addr, byte ptr[], byte length)
{
    if(length!=0)
    {
        if(length==1)
        {
            SPI_CS_pin(GPIO_PIN_RESET);
            bSpiTransfer(addr);
            ptr[0] = bSpiTransfer(0xFF);
            SPI_CS_pin(GPIO_PIN_SET);
        }
        else
        {
            byte i;
            SPI_CS_pin(GPIO_PIN_RESET);
            bSpiTransfer(addr);
            for(i=0; i<length; i++)
                ptr[i] = bSpiTransfer(0xFF);
            SPI_CS_pin(GPIO_PIN_SET);
        }
    }
    return;
}

inline static void SPI_CS_pin(GPIO_PinState _PinState)
{
    HAL_GPIO_WritePin(SPI_CS_GPIOx, SPI_CS_GPIO_Pin, _PinState);
}

inline static void SetPOR(void)
{
    HAL_GPIO_WritePin(RESET_GPIOx, RESET_GPIO_Pin, GPIO_PIN_SET);
}
inline static void ClrPOR(void)
{
    HAL_GPIO_WritePin(RESET_GPIOx, RESET_GPIO_Pin, GPIO_PIN_RESET);
}

static uint8_t DIO0_H(RFM98_LoRa_data_t *_data)
{
    if( _data->interruptFlag.bit.DIO0 )
    {
        _data->interruptFlag.bit.DIO0 = false;
        return true;
    }
    return false;
}

#if 0
/**
  * @brief  Transmit byte to register.
  * @param  addr: address
  * @param  value: register value.
  * @retval HAL status
  */
static uint8_t WriteRegister(uint8_t addr, uint8_t value)
{
    uint8_t data[2] = {0, 0};
    uint8_t status = 0;
    SPI_CS_pin(GPIO_PIN_RESET);
    data[0] = (addr | 0x80);
#if 0
    status = HAL_SPI_Transmit(hspi_RFM98W, data, 1, SPI_TIMEOUT);
    status = HAL_SPI_Transmit(hspi_RFM98W, &value, 1, SPI_TIMEOUT);
#else
    data[0] = (addr | 0x80);
    data[1] = value;
    status = HAL_SPI_Transmit(hspi_RFM98W, data, 2, SPI_TIMEOUT);
#endif
    SPI_CS_pin(GPIO_PIN_SET);
    return status;
}

/**
  * @brief  Read byte from register.
  * @param  addr: address
  * @retval register value.
  */
static uint8_t ReadRegister(uint8_t addr)
{
    uint8_t data[1] = {0};
    SPI_CS_pin(GPIO_PIN_RESET);
    data[0] = (addr & 0x7F);
    HAL_SPI_Transmit(hspi_RFM98W, data, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(hspi_RFM98W, data, 1, SPI_TIMEOUT);
    SPI_CS_pin(GPIO_PIN_SET);
    return data[0];
}
#endif
