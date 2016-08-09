/*
  / _____)             _              | |
  ( (____  _____ ____ _| |_ _____  ____| |__
  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
  _____) ) ____| | | || |_| ____( (___| | | |
  (______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech

  Description: LoRaMac classA device implementation

  License: Revised BSD License, see LICENSE.TXT file include in the project

  Maintainer: Miguel Luis and Gregory Cristian
*/
#include <cox.h>

LoRaMac &LoRaWAN = enableLoRaMac();

#define OVER_THE_AIR_ACTIVATION 1

#if (OVER_THE_AIR_ACTIVATION == 1)
static const uint8_t devEui[] = "\xC2\xAE\x00\x00\x00\x00\x80\x00";

static const uint8_t appEui[] = "\x1A\x00\x00\xFE\x00\x00\x00\x00";

static const uint8_t appKey[] = "\x0B\xF2\x80\x34\xED\xCB\x14\xE0\x9E\x1F\x94\xEA\x73\xE8\xEF\x0E";

#else

static uint8_t NwkSKey[] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

static uint8_t AppSKey[] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

static uint32_t DevAddr = 0;
#endif //OVER_THE_AIR_ACTIVATION

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

static uint8_t AppPort = 2;
static uint8_t AppDataSize = 16;
static uint8_t AppData[64];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = true;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static Timer TxNextPacketTimer;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame(uint8_t port) {
  if (port == 2) {
    uint16_t pressure = 0;
    int16_t altitudeBar = 0;
    int16_t temperature = 0;
    int32_t latitude = 0, longitude = 0;
    uint16_t altitudeGps = 0xFFFF;
    uint8_t batteryLevel = 0;

    AppData[0] = 0;
    AppData[1] = ( pressure >> 8 ) & 0xFF;
    AppData[2] = pressure & 0xFF;
    AppData[3] = ( temperature >> 8 ) & 0xFF;
    AppData[4] = temperature & 0xFF;
    AppData[5] = ( altitudeBar >> 8 ) & 0xFF;
    AppData[6] = altitudeBar & 0xFF;
    AppData[7] = batteryLevel;
    AppData[8] = ( latitude >> 16 ) & 0xFF;
    AppData[9] = ( latitude >> 8 ) & 0xFF;
    AppData[10] = latitude & 0xFF;
    AppData[11] = ( longitude >> 16 ) & 0xFF;
    AppData[12] = ( longitude >> 8 ) & 0xFF;
    AppData[13] = longitude & 0xFF;
    AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
    AppData[15] = altitudeGps & 0xFF;
  }
}


/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent(void *) {
  LoRaMac::MibRequestConfirm_t mibReq;
  LoRaMac::Status_t status;

  mibReq.Type = LoRaMac::MIB_NETWORK_JOINED;
  status = LoRaWAN.mibGetRequestConfirm(&mibReq);

  if (status == LoRaMac::STATUS_OK) {
    if (mibReq.Param.IsNetworkJoined) {
      LoRaMac::McpsReq_t mcpsReq;
      LoRaMac::TxInfo_t txInfo;

      PrepareTxFrame(AppPort);
      printf("%s()-prepare done\n", __func__);

      if (LoRaWAN.isTxPossible(AppDataSize, &txInfo) != LoRaMac::STATUS_OK) {
        printf("%s()-Tx impossible\n", __func__);
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = LoRaMac::MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = 0;
      } else {
        if (IsTxConfirmed == false) {
          printf("%s()-Tx Unconfirmed\n", __func__);
          mcpsReq.Type = LoRaMac::MCPS_UNCONFIRMED;
          mcpsReq.Req.Unconfirmed.fPort = AppPort;
          mcpsReq.Req.Unconfirmed.fBuffer = AppData;
          mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
          mcpsReq.Req.Unconfirmed.Datarate = 0;
        } else {
          printf("%s()-Tx Confirmed\n", __func__);
          mcpsReq.Type = LoRaMac::MCPS_CONFIRMED;
          mcpsReq.Req.Confirmed.fPort = AppPort;
          mcpsReq.Req.Confirmed.fBuffer = AppData;
          mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
          mcpsReq.Req.Confirmed.NbTrials = 8;
          mcpsReq.Req.Confirmed.Datarate = 0;
        }
      }
      LoRaWAN.mcpsRequest(&mcpsReq);
    } else {
#if (OVER_THE_AIR_ACTIVATION == 1)
      printf("%s()-Retry join\n", __func__);
      LoRaMac::MlmeReq_t mlmeReq;
      mlmeReq.Type = LoRaMac::MLME_JOIN;
      mlmeReq.Req.Join.DevEui = devEui;
      mlmeReq.Req.Join.AppEui = appEui;
      mlmeReq.Req.Join.AppKey = appKey;
      LoRaWAN.mlmeRequest(&mlmeReq);
#endif
    }
  }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm(LoRaMac::McpsConfirm_t *McpsConfirm) {
  printf("%s()\n", __func__);
  if (McpsConfirm->Status == LoRaMac::EVENT_INFO_STATUS_OK) {
    if (McpsConfirm->McpsRequest == LoRaMac::MCPS_UNCONFIRMED) {
      printf("%s()-unconfirmed\n", __func__);
      // Check Datarate
      // Check TxPower
    } else if (McpsConfirm->McpsRequest == LoRaMac::MCPS_CONFIRMED) {
      printf("%s()-confirmed\n", __func__);
      // Check Datarate
      // Check TxPower
      // Check AckReceived
      // Check NbTrials
    } else if (McpsConfirm->McpsRequest == LoRaMac::MCPS_PROPRIETARY) {
      printf("%s()-proprietary\n", __func__);
    } else {
    }
  }

  // Schedule next packet transmission
  uint32_t nextTime = APP_TX_DUTYCYCLE + random(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
  TxNextPacketTimer.onFired(OnTxNextPacketTimerEvent, NULL);
  TxNextPacketTimer.startOneShot(nextTime);
  printf("%s()-new duty cycle time:%lu\n", __func__, nextTime);
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] McpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication(LoRaMac::McpsIndication_t *McpsIndication) {
  if (McpsIndication->Status != LoRaMac::EVENT_INFO_STATUS_OK) {
    return;
  }

  if (McpsIndication->McpsIndication == LoRaMac::MCPS_UNCONFIRMED) {
  } else if (McpsIndication->McpsIndication == LoRaMac::MCPS_CONFIRMED) {
  } else if (McpsIndication->McpsIndication == LoRaMac::MCPS_PROPRIETARY) {
  } else if (McpsIndication->McpsIndication == LoRaMac::MCPS_MULTICAST) {
  } else {
  }

  // Check Multicast
  // Check Port
  // Check Datarate
  // Check FramePending
  // Check Buffer
  // Check BufferSize
  // Check Rssi
  // Check Snr
  // Check RxSlot

  if (McpsIndication->RxData == true) {
    uint8_t x;
    printf("[%lu usec] Rx port:%u, len:%u,", micros(), McpsIndication->Port, McpsIndication->BufferSize);
    for (x = 0; x < McpsIndication->BufferSize; x++) {
      printf(" %02X", McpsIndication->Buffer[x]);
    }
    printf("\n");
  }
}

static void MlmeConfirm(LoRaMac::MlmeConfirm_t *MlmeConfirm) {
  if (MlmeConfirm->MlmeRequest == LoRaMac::MLME_JOIN) {
    if (MlmeConfirm->Status == LoRaMac::EVENT_INFO_STATUS_OK) {
      // Status is OK, node has joined the network
      printf("%s()-joined\n", __func__);
      postTask(OnTxNextPacketTimerEvent, NULL);
    } else {
      printf("%s()-join failed. Retry to join\n", __func__);
      LoRaMac::MlmeReq_t mlmeReq;
      mlmeReq.Type = LoRaMac::MLME_JOIN;
      mlmeReq.Req.Join.DevEui = devEui;
      mlmeReq.Req.Join.AppEui = appEui;
      mlmeReq.Req.Join.AppKey = appKey;
      LoRaWAN.mlmeRequest(&mlmeReq);
    }
  } else if (MlmeConfirm->MlmeRequest == LoRaMac::MLME_LINK_CHECK)  {
    printf("%s()-link check\n", __func__);
    // Check DemodMargin
    // Check NbGateways
  } else {
  }

}

void setup() {
  Serial.begin(115200);
  Serial.printf("\n*** LoRaWAN Class A Example ***\n");

  LoRaMac::Primitives_t LoRaMacPrimitives;
  LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
  LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
  LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;

  LoRaMac::Callback_t LoRaMacCallbacks;
  LoRaMacCallbacks.GetBatteryLevel = NULL;

  LoRaWAN.begin(SX1276, &LoRaMacPrimitives, &LoRaMacCallbacks);

#if (OVER_THE_AIR_ACTIVATION == 0)
  printf("ABP!\n");
  LoRaMac::MibRequestConfirm_t mibReq;

  mibReq.Type = LoRaMac::MIB_ADR;
  mibReq.Param.AdrEnable = 1;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  mibReq.Type = LoRaMac::MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = true;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  // Choose a random device address
  DevAddr = random(0, 0x01FFFFFF);

  mibReq.Type = LoRaMac::MIB_NET_ID;
  mibReq.Param.NetID = 0x34;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  mibReq.Type = LoRaMac::MIB_DEV_ADDR;
  mibReq.Param.DevAddr = DevAddr;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  mibReq.Type = LoRaMac::MIB_NWK_SKEY;
  mibReq.Param.NwkSKey = NwkSKey;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  mibReq.Type = LoRaMac::MIB_APP_SKEY;
  mibReq.Param.AppSKey = AppSKey;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  mibReq.Type = LoRaMac::MIB_NETWORK_JOINED;
  mibReq.Param.IsNetworkJoined = true;
  LoRaWAN.mibSetRequestConfirm(&mibReq);

  postTask(OnTxNextPacketTimerEvent, NULL);
#else
  printf("Trying to join\n");
  LoRaMac::MlmeReq_t mlmeReq;
  mlmeReq.Type = LoRaMac::MLME_JOIN;
  mlmeReq.Req.Join.DevEui = devEui;
  mlmeReq.Req.Join.AppEui = appEui;
  mlmeReq.Req.Join.AppKey = appKey;
  LoRaWAN.mlmeRequest(&mlmeReq);
#endif
}