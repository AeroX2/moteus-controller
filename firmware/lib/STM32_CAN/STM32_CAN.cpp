#include "STM32_CAN.h"

constexpr Baudrate_entry_t STM32_CAN::BAUD_RATE_TABLE_48M[];
constexpr Baudrate_entry_t STM32_CAN::BAUD_RATE_TABLE_45M[];

static STM32_CAN* _CAN1 = nullptr;
static FDCAN_HandleTypeDef     hcan1;
uint32_t test = 0;

STM32_CAN::STM32_CAN( FDCAN_GlobalTypeDef* canPort, CAN_PINS pins, RXQUEUE_TABLE rxSize, TXQUEUE_TABLE txSize ) {

  if (_canIsActive) { return; }

  sizeRxBuffer=rxSize;
  sizeTxBuffer=txSize;

  if (canPort == FDCAN1)
  {
    _CAN1 = this;
    n_pCanHandle = &hcan1;
  }

  _canPort = canPort;
  _pins = pins;
}

  struct Rate {
    int prescaler = -1;
    int sync_jump_width = -1;
    int time_seg1 = -1;
    int time_seg2 = -1;
  };


Rate MakeTime(int bitrate, int max_time_seg1, int max_time_seg2) {
  Rate result;

  result.prescaler = 1;

  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t total_divisor = 0;

  while (true) {
    total_divisor = (pclk1 / result.prescaler) / bitrate;

    // One of the divisor counts comes for free.
    const auto actual_divisor = total_divisor - 1;

    // Split up the remainder roughly 3/1
    result.time_seg2 = actual_divisor / 3;
    result.time_seg1 = actual_divisor - result.time_seg2;

    result.sync_jump_width = std::min(16, result.time_seg2);

    if (result.time_seg1 > max_time_seg1 ||
        result.time_seg2 > max_time_seg2) {
      result.prescaler++;
      continue;
    }

    break;
  }

  return result;
}

// Init and start CAN
void STM32_CAN::begin( bool retransmission ) {

  // exit if CAN already is active
  if (_canIsActive) return;

  _canIsActive = true;

  GPIO_InitTypeDef GPIO_InitStruct;

  initializeBuffers();

  // Configure CAN
  if (_canPort == FDCAN1)
  {
    //CAN1

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
      
    __HAL_RCC_FDCAN_CLK_ENABLE();

    // if (_pins == ALT)
    // {
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // }
    // if (_pins == DEF)
    // {
    //   __HAL_RCC_GPIOA_CLK_ENABLE();
    //   GPIO_InitStruct.Pull = GPIO_NOPULL;
    //   #if defined(__HAL_RCC_AFIO_CLK_ENABLE)
    //   __HAL_AFIO_REMAP_CAN1_1(); // To use PA11/12 pins for CAN1.
    //   __HAL_RCC_AFIO_CLK_ENABLE();
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   GPIO_InitStruct.Pin = GPIO_PIN_11;
    //   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    //   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //   GPIO_InitStruct.Pin = GPIO_PIN_12;
    //   #else
    //   #if defined(GPIO_AF4_CAN)
    //   GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    //   #else
    //   GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    //   #endif
    //   GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    //   #if defined(GPIO_SPEED_FREQ_VERY_HIGH)
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //   #else
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   #endif
    //   #endif
    //   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // }

    // #if defined(__HAL_RCC_GPIOD_CLK_ENABLE) // not all MCU variants have port GPIOD available
    // if (_pins == ALT_2)
    // {
    //   __HAL_RCC_GPIOD_CLK_ENABLE();
    //   GPIO_InitStruct.Pull = GPIO_NOPULL;
    //   #if defined(__HAL_RCC_AFIO_CLK_ENABLE)
    //   __HAL_AFIO_REMAP_CAN1_3(); // To use PD0/1 pins for CAN1.
    //   __HAL_RCC_AFIO_CLK_ENABLE();
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   GPIO_InitStruct.Pin = GPIO_PIN_0;
    //   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    //   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    //   GPIO_InitStruct.Pin = GPIO_PIN_1;
    //   #else
    //   #if defined(GPIO_AF4_CAN)
    //   GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    //   #else
    //   GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    //   #endif
    //   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    //   #if defined(GPIO_SPEED_FREQ_VERY_HIGH)
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //   #else
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   #endif
    //   #endif
    //   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    // }
    // #endif

    // NVIC configuration for CAN1 Reception complete interrupt
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 15, 0); // 15 is lowest possible priority
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    // NVIC configuration for CAN1 Transmission complete interrupt
    // HAL_NVIC_SetPriority(CAN1_TX_IRQn, 15, 0); // 15 is lowest possible priority
    // HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

    n_pCanHandle->Instance = FDCAN1;
  }

  n_pCanHandle->Init.ClockDivider = FDCAN_CLOCK_DIV1;
  n_pCanHandle->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  n_pCanHandle->Init.Mode = FDCAN_MODE_NORMAL;

  if (retransmission){ n_pCanHandle->Init.AutoRetransmission  = ENABLE; }
  else { n_pCanHandle->Init.AutoRetransmission  = DISABLE; }

  // auto nominal = MakeTime(1000000, 255, 127);
  // auto fast = MakeTime(5000000, 31, 15);

  n_pCanHandle->Init.TransmitPause = ENABLE;
  n_pCanHandle->Init.ProtocolException = DISABLE;
  n_pCanHandle->Init.NominalPrescaler = 10; // nominal.prescaler;
  n_pCanHandle->Init.NominalSyncJumpWidth = 1; //nominal.sync_jump_width;
  n_pCanHandle->Init.NominalTimeSeg1 = 14; // nominal.time_seg1;
  n_pCanHandle->Init.NominalTimeSeg2 = 2; //nominal.time_seg2;
  n_pCanHandle->Init.DataPrescaler = 1; //fast.prescaler;
  n_pCanHandle->Init.DataSyncJumpWidth = 4; // fast.sync_jump_width;
  n_pCanHandle->Init.DataTimeSeg1 = 5;//fast.time_seg1;
  n_pCanHandle->Init.DataTimeSeg2 = 4;//fast.time_seg2;
  n_pCanHandle->Init.StdFiltersNbr = 1;
  n_pCanHandle->Init.ExtFiltersNbr = 0;
  n_pCanHandle->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
}


void STM32_CAN::setBaudRate(uint32_t baud)
{

  // Calculate and set baudrate
  // calculateBaudrate( n_pCanHandle, baud );

  // Initializes CAN
  HAL_FDCAN_Init( n_pCanHandle );

  initializeFilters();

  HAL_FDCAN_ConfigGlobalFilter(n_pCanHandle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // HAL_FDCAN_ConfigTxDelayCompensation(n_pCanHandle, 13, 1);
  // HAL_FDCAN_EnableTxDelayCompensation(n_pCanHandle);

  // Start the CAN peripheral
  HAL_FDCAN_Start( n_pCanHandle );

  // Activate CAN RX notification
  HAL_FDCAN_ActivateNotification( n_pCanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // TODO
  // // Activate CAN TX notification
  // HAL_FDCAN_ActivateNotification( n_pCanHandle, FDCAN_IT_TX_FIFO_EMPTY, 0);
}

bool STM32_CAN::write(CAN_message_t &CAN_tx_msg, bool sendMB)
{
  bool ret = true;
  uint32_t TxMailbox;
  FDCAN_TxHeaderTypeDef TxHeader;

  __HAL_FDCAN_DISABLE_IT(n_pCanHandle, FDCAN_IT_TX_FIFO_EMPTY);

  if (CAN_tx_msg.flags.extended == 1) // Extended ID when CAN_tx_msg.flags.extended is 1
  {
      TxHeader.Identifier = CAN_tx_msg.id;
      TxHeader.IdType   = FDCAN_EXTENDED_ID;
  }
  else // Standard ID otherwise
  {
      TxHeader.Identifier = CAN_tx_msg.id;
      TxHeader.IdType   = FDCAN_STANDARD_ID;
  }

  if (CAN_tx_msg.flags.remote == 1) // Remote frame when CAN_tx_msg.flags.remote is 1
  {
    TxHeader.TxFrameType   = FDCAN_REMOTE_FRAME;
    TxHeader.DataLength   = 0;
  }
  else{
    TxHeader.TxFrameType   = FDCAN_DATA_FRAME;
    TxHeader.DataLength   = ((uint32_t)(CAN_tx_msg.len)) << 16;
  }

  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if(HAL_FDCAN_AddMessageToTxFifoQ( n_pCanHandle, &TxHeader, CAN_tx_msg.buf) != HAL_OK)
  {
    /* in normal situation we add up the message to TX ring buffer, if there is no free TX mailbox. But the TX mailbox interrupt is using this same function
    to move the messages from ring buffer to empty TX mailboxes, so for that use case, there is this check */
    if(sendMB != true)
    {
      if( addToRingBuffer(txRing, CAN_tx_msg) == false )
      {
        ret = false; // no more room
      }
    }
    else { ret = false; }
  }
  __HAL_FDCAN_ENABLE_IT(n_pCanHandle, FDCAN_IT_TX_FIFO_EMPTY);
  return ret;
}

bool STM32_CAN::read(CAN_message_t &CAN_rx_msg)
{
  bool ret;
  __HAL_FDCAN_DISABLE_IT(n_pCanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  ret = removeFromRingBuffer(rxRing, CAN_rx_msg);
  __HAL_FDCAN_ENABLE_IT(n_pCanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  return ret;
}

bool STM32_CAN::setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, IDE std_ext)
{
  FDCAN_FilterTypeDef sFilterConfig;

  // sFilterConfig.FilterBank = bank_num;
  // sFilterConfig.FilterMode = filter_mode;
  // sFilterConfig.FilterScale = filter_scale;
  // sFilterConfig.FilterFIFOAssignment = fifo;
  // sFilterConfig.FilterActivation = ENABLE;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  if (std_ext == STD || (std_ext == AUTO && filter_id <= 0x7FF))
  {
    // Standard ID can be only 11 bits long
    // sFilterConfig.FilterIdHigh = (uint16_t) (filter_id << 5);
    // sFilterConfig.FilterIdLow = 0;
    // sFilterConfig.FilterMaskIdHigh = (uint16_t) (mask << 5);
    // sFilterConfig.FilterMaskIdLow = CAN_ID_EXT;

    sFilterConfig.FilterID1 = filter_id;
    sFilterConfig.FilterID2 = 0x7FF;
  }
  else
  {
    // Extended ID
    // sFilterConfig.FilterIdLow = (uint16_t) (filter_id << 3);
    // sFilterConfig.FilterIdLow |= CAN_ID_EXT;
    // sFilterConfig.FilterIdHigh = (uint16_t) (filter_id >> 13);
    // sFilterConfig.FilterMaskIdLow = (uint16_t) (mask << 3);
    // sFilterConfig.FilterMaskIdLow |= CAN_ID_EXT;
    // sFilterConfig.FilterMaskIdHigh = (uint16_t) (mask >> 13);

    sFilterConfig.FilterID1 = filter_id;
    sFilterConfig.FilterID2 = 0x7FF;
  }

  // Enable filter
  if (HAL_FDCAN_ConfigFilter( n_pCanHandle, &sFilterConfig ) != HAL_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

// void STM32_CAN::setMBFilter(CAN_BANK bank_num, CAN_FLTEN input)
// {
//   FDCAN_FilterTypeDef sFilterConfig;
//   sFilterConfig.FilterBank = uint8_t(bank_num);
//   if (input == ACCEPT_ALL) { sFilterConfig.FilterActivation = ENABLE; }
//   else { sFilterConfig.FilterActivation = DISABLE; }

//   HAL_FDCAN_ConfigFilter(n_pCanHandle, &sFilterConfig);
// }

// void STM32_CAN::setMBFilter(CAN_FLTEN input)
// {
//   FDCAN_FilterTypeDef sFilterConfig;
//   uint8_t max_bank_num = 27;
//   uint8_t min_bank_num = 0;

//   for (uint8_t bank_num = min_bank_num ; bank_num <= max_bank_num ; bank_num++)
//   {
//     sFilterConfig.FilterBank = bank_num;
//     if (input == ACCEPT_ALL) { sFilterConfig.FilterActivation = ENABLE; }
//     else { sFilterConfig.FilterActivation = DISABLE; }
//     HAL_FDCAN_ConfigFilter(n_pCanHandle, &sFilterConfig);
//   }
// }

// bool STM32_CAN::setMBFilterProcessing(CAN_BANK bank_num, uint32_t filter_id, uint32_t mask, IDE std_ext)
// {
//   // just convert the MB number enum to bank number.
//   return setFilter(uint8_t(bank_num), filter_id, mask, std_ext);
// }

// bool STM32_CAN::setMBFilter(CAN_BANK bank_num, uint32_t id1, IDE std_ext)
// {
//   // by setting the mask to 0x1FFFFFFF we only filter the ID set as Filter ID.
//   return setFilter(uint8_t(bank_num), id1, 0x1FFFFFFF, std_ext);
// }

// bool STM32_CAN::setMBFilter(CAN_BANK bank_num, uint32_t id1, uint32_t id2, IDE std_ext)
// {
//   // if we set the filter mode as IDLIST, the mask becomes filter ID too. So we can filter two totally independent IDs in same bank.
//   return setFilter(uint8_t(bank_num), id1, id2, AUTO, CAN_FILTERMODE_IDLIST, std_ext);
// }

// TBD, do this using "setFilter" -function
void STM32_CAN::initializeFilters()
{
  FDCAN_FilterTypeDef sFilterConfig;
  // We set first bank to accept all RX messages
  // sFilterConfig.FilterBank = 0;
  // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  // sFilterConfig.FilterIdHigh = 0x0000;
  // sFilterConfig.FilterIdLow = 0x0000;
  // sFilterConfig.FilterMaskIdHigh = 0x0000;
  // sFilterConfig.FilterMaskIdLow = 0x0000;
  // sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  // sFilterConfig.FilterActivation = ENABLE;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0;
  sFilterConfig.FilterID2 = 0;

  HAL_FDCAN_ConfigFilter(n_pCanHandle, &sFilterConfig);
}

void STM32_CAN::initializeBuffers()
{
    if(isInitialized()) { return; }

    // set up the transmit and receive ring buffers
    if(tx_buffer==0)
    {
      tx_buffer=new CAN_message_t[sizeTxBuffer];
    }
    initRingBuffer(txRing, tx_buffer, sizeTxBuffer);

    if(rx_buffer==0)
    {
      rx_buffer=new CAN_message_t[sizeRxBuffer];
    }
    initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}

void STM32_CAN::initRingBuffer(RingbufferTypeDef &ring, volatile CAN_message_t *buffer, uint32_t size)
{
    ring.buffer = buffer;
    ring.size = size;
    ring.head = 0;
    ring.tail = 0;
}

bool STM32_CAN::addToRingBuffer(RingbufferTypeDef &ring, const CAN_message_t &msg)
{
    uint16_t nextEntry;
    nextEntry =(ring.head + 1) % ring.size;

    // check if the ring buffer is full
    if(nextEntry == ring.tail)
	{
        return(false);
    }

    // add the element to the ring */
    memcpy((void *)&ring.buffer[ring.head],(void *)&msg, sizeof(CAN_message_t));

    // bump the head to point to the next free entry
    ring.head = nextEntry;

    return(true);
}
bool STM32_CAN::removeFromRingBuffer(RingbufferTypeDef &ring, CAN_message_t &msg)
{
    // check if the ring buffer has data available
    if(isRingBufferEmpty(ring) == true)
    {
        return(false);
    }

    // copy the message
    memcpy((void *)&msg,(void *)&ring.buffer[ring.tail], sizeof(CAN_message_t));

    // bump the tail pointer
    ring.tail =(ring.tail + 1) % ring.size;
    return(true);
}

bool STM32_CAN::isRingBufferEmpty(RingbufferTypeDef &ring)
{
    if(ring.head == ring.tail)
	{
        return(true);
    }

    return(false);
}

uint32_t STM32_CAN::ringBufferCount(RingbufferTypeDef &ring)
{
    int32_t entries;
    entries = ring.head - ring.tail;

    if(entries < 0)
    {
        entries += ring.size;
    }
    return((uint32_t)entries);
}

// void STM32_CAN::setBaudRateValues(FDCAN_HandleTypeDef *CanHandle, uint16_t prescaler, uint8_t timeseg1,
//                                                                 uint8_t timeseg2, uint8_t sjw)
// {
//   uint32_t _SyncJumpWidth = 0;
//   uint32_t _TimeSeg1 = 0;
//   uint32_t _TimeSeg2 = 0;
//   uint32_t _Prescaler = 0;
//   switch (sjw)
//   {
//     case 1:
//       _SyncJumpWidth = CAN_SJW_1TQ;
//       break;
//     case 2:
//       _SyncJumpWidth = CAN_SJW_2TQ;
//       break;
//     case 3:
//       _SyncJumpWidth = CAN_SJW_3TQ;
//       break;
//     case 4:
//       _SyncJumpWidth = CAN_SJW_4TQ;
//       break;
//     default:
//       // should not happen
//       _SyncJumpWidth = CAN_SJW_1TQ;
//       break;
//   }

//   switch (timeseg1)
//   {
//     case 1:
//       _TimeSeg1 = CAN_BS1_1TQ;
//       break;
//     case 2:
//       _TimeSeg1 = CAN_BS1_2TQ;
//       break;
//     case 3:
//       _TimeSeg1 = CAN_BS1_3TQ;
//       break;
//     case 4:
//       _TimeSeg1 = CAN_BS1_4TQ;
//       break;
//     case 5:
//       _TimeSeg1 = CAN_BS1_5TQ;
//       break;
//     case 6:
//       _TimeSeg1 = CAN_BS1_6TQ;
//       break;
//     case 7:
//       _TimeSeg1 = CAN_BS1_7TQ;
//       break;
//     case 8:
//       _TimeSeg1 = CAN_BS1_8TQ;
//       break;
//     case 9:
//       _TimeSeg1 = CAN_BS1_9TQ;
//       break;
//     case 10:
//       _TimeSeg1 = CAN_BS1_10TQ;
//       break;
//     case 11:
//       _TimeSeg1 = CAN_BS1_11TQ;
//       break;
//     case 12:
//       _TimeSeg1 = CAN_BS1_12TQ;
//       break;
//     case 13:
//       _TimeSeg1 = CAN_BS1_13TQ;
//       break;
//     case 14:
//       _TimeSeg1 = CAN_BS1_14TQ;
//       break;
//     case 15:
//       _TimeSeg1 = CAN_BS1_15TQ;
//       break;
//     case 16:
//       _TimeSeg1 = CAN_BS1_16TQ;
//       break;
//     default:
//       // should not happen
//       _TimeSeg1 = CAN_BS1_1TQ;
//       break;
//   }

//   switch (timeseg2)
//   {
//     case 1:
//       _TimeSeg2 = CAN_BS2_1TQ;
//       break;
//     case 2:
//       _TimeSeg2 = CAN_BS2_2TQ;
//       break;
//     case 3:
//       _TimeSeg2 = CAN_BS2_3TQ;
//       break;
//     case 4:
//       _TimeSeg2 = CAN_BS2_4TQ;
//       break;
//     case 5:
//       _TimeSeg2 = CAN_BS2_5TQ;
//       break;
//     case 6:
//       _TimeSeg2 = CAN_BS2_6TQ;
//       break;
//     case 7:
//       _TimeSeg2 = CAN_BS2_7TQ;
//       break;
//     case 8:
//       _TimeSeg2 = CAN_BS2_8TQ;
//       break;
//     default:
//       // should not happen
//       _TimeSeg2 = CAN_BS2_1TQ;
//       break;
//   }
//   _Prescaler = prescaler;

//   CanHandle->Init.SyncJumpWidth = _SyncJumpWidth;
//   CanHandle->Init.TimeSeg1 = _TimeSeg1;
//   CanHandle->Init.TimeSeg2 = _TimeSeg2;
//   CanHandle->Init.Prescaler = _Prescaler;
// }

// void STM32_CAN::calculateBaudrate(FDCAN_HandleTypeDef *CanHandle, int baud)
// {
//   /* this function calculates the needed Sync Jump Width, Time segments 1 and 2 and prescaler values based on the set baud rate and APB1 clock.
//   This could be done faster if needed by calculating these values beforehand and just using fixed values from table.
//   The function has been optimized to give values that have sample-point between 75-94%. If some other sample-point percentage is needed, this needs to be adjusted.
//   More info about this topic here: http://www.bittiming.can-wiki.info/
//   */
//   int sjw = 1;
//   int bs1 = 5; // optimization. bs1 smaller than 5 does give too small sample-point percentages.
//   int bs2 = 1;
//   int prescaler = 1;
//   uint16_t i = 0;

//   uint32_t frequency = getAPB1Clock();

//   if(frequency == 48000000) {
//     for(i=0; i<sizeof(BAUD_RATE_TABLE_48M)/sizeof(Baudrate_entry_t); i++) {
//       if(baud == (int)BAUD_RATE_TABLE_48M[i].baudrate) {
//         break;
//       }
//     }
//     if(i < sizeof(BAUD_RATE_TABLE_48M)/sizeof(Baudrate_entry_t)) {
//       setBaudRateValues(CanHandle, BAUD_RATE_TABLE_48M[i].prescaler,
//                                    BAUD_RATE_TABLE_48M[i].timeseg1,
//                                    BAUD_RATE_TABLE_48M[i].timeseg2,
//                                    1);
//       return;
//     }
//   }
//   else if(frequency == 45000000) {
//     for(i=0; i<sizeof(BAUD_RATE_TABLE_45M)/sizeof(Baudrate_entry_t); i++) {
//       if(baud == (int)BAUD_RATE_TABLE_45M[i].baudrate) {
//         break;
//       }
//     }
//     if(i < sizeof(BAUD_RATE_TABLE_45M)/sizeof(Baudrate_entry_t)) {
//       setBaudRateValues(CanHandle, BAUD_RATE_TABLE_45M[i].prescaler,
//                                    BAUD_RATE_TABLE_45M[i].timeseg1,
//                                    BAUD_RATE_TABLE_45M[i].timeseg2,
//                                    1);
//       return;
//     }
//   }

//   while (sjw <= 4) {
//     while (prescaler <= 1024) {
//       while (bs2 <= 3) { // Time segment 2 can get up to 8, but that causes too small sample-point percentages, so this is limited to 3.
//         while (bs1 <= 15) { // Time segment 1 can get up to 16, but that causes too big sample-point percenages, so this is limited to 15.
//           int calcBaudrate = (int)(frequency / (prescaler * (sjw + bs1 + bs2)));

//           if (calcBaudrate == baud)
//           {
//             setBaudRateValues(CanHandle, prescaler, bs1, bs2, sjw);
//             return;
//           }
//           bs1++;
//         }
//         bs1 = 5;
//         bs2++;
//       }
//       bs1 = 5;
//       bs2 = 1;
//       prescaler++;
//     }
//     bs1 = 5;
//     sjw++;
//   }
// }

// uint32_t STM32_CAN::getAPB1Clock()
// {
//   RCC_ClkInitTypeDef clkInit;
//   uint32_t flashLatency;
//   HAL_RCC_GetClockConfig(&clkInit, &flashLatency);

//   uint32_t hclkClock = HAL_RCC_GetHCLKFreq();
//   uint8_t clockDivider = 1;
//   switch (clkInit.APB1CLKDivider)
//   {
//   case RCC_HCLK_DIV1:
//     clockDivider = 1;
//     break;
//   case RCC_HCLK_DIV2:
//     clockDivider = 2;
//     break;
//   case RCC_HCLK_DIV4:
//     clockDivider = 4;
//     break;
//   case RCC_HCLK_DIV8:
//     clockDivider = 8;
//     break;
//   case RCC_HCLK_DIV16:
//     clockDivider = 16;
//     break;
//   default:
//     // should not happen
//     break;
//   }

//   uint32_t apb1Clock = hclkClock / clockDivider;

//   return apb1Clock;
// }

// void STM32_CAN::enableMBInterrupts()
// {
//   if (n_pCanHandle->Instance == CAN1)
//   {
//     HAL_NVIC_EnableIRQ(FDCAN1_TX_IRQn);
//   }
// #ifdef CAN2
//   else if (n_pCanHandle->Instance == CAN2)
//   {
//     HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
//   }
// #endif
// #ifdef CAN3
//   else if (n_pCanHandle->Instance == CAN3)
//   {
//     HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
//   }
// #endif
// }

// void STM32_CAN::disableMBInterrupts()
// {
//   if (n_pCanHandle->Instance == CAN1)
//   {
//     HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
//   }
// #ifdef CAN2
//   else if (n_pCanHandle->Instance == CAN2)
//   {
//     HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
//   }
// #endif
// #ifdef CAN3
//   else if (n_pCanHandle->Instance == CAN3)
//   {
//     HAL_NVIC_DisableIRQ(CAN3_TX_IRQn);
//   }
// #endif
// }

void STM32_CAN::enableLoopBack( bool yes ) {
  if (yes) { n_pCanHandle->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK; }
  else { n_pCanHandle->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK; }
}

// void STM32_CAN::enableSilentMode( bool yes ) {
//   if (yes) { n_pCanHandle->Init.Mode = CAN_MODE_SILENT; }
//   else { n_pCanHandle->Init.Mode = CAN_MODE_NORMAL; }
// }

// void STM32_CAN::enableSilentLoopBack( bool yes ) {
//   if (yes) { n_pCanHandle->Init.Mode = CAN_MODE_SILENT_LOOPBACK; }
//   else { n_pCanHandle->Init.Mode = CAN_MODE_NORMAL; }
// }

void STM32_CAN::enableFIFO(bool status)
{
  //Nothing to do here. The FIFO is on by default. This is just to work with code made for Teensy FlexCan.
  (void) status;
}

/* Interrupt functions
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
// // There is 3 TX mailboxes. Each one has own transmit complete callback function, that we use to pull next message from TX ringbuffer to be sent out in TX mailbox.
// extern "C" void HAL_CAN_TxMailbox0CompleteCallback( FDCAN_HandleTypeDef *CanHandle )
// {
//   CAN_message_t txmsg;
//   // use correct CAN instance
//   if (CanHandle->Instance == FDCAN1) 
//   {
//     if (_CAN1->removeFromRingBuffer(_CAN1->txRing, txmsg))
//     {
//       _CAN1->write(txmsg, true);
//     }
//   }
// #ifdef CAN2
//   else if (CanHandle->Instance == CAN2) 
//   {
//     if (_CAN2->removeFromRingBuffer(_CAN2->txRing, txmsg))
//     {
//       _CAN2->write(txmsg, true);
//     }
//   }
// #endif
// #ifdef CAN3
//   else if (CanHandle->Instance == CAN3) 
//   {
//     if (_CAN3->removeFromRingBuffer(_CAN3->txRing, txmsg))
//     {
//       _CAN3->write(txmsg, true);
//     }
//   }
// #endif
// }

// extern "C" void HAL_CAN_TxMailbox1CompleteCallback( FDCAN_HandleTypeDef *CanHandle )
// {
//   CAN_message_t txmsg;
//   // use correct CAN instance
//   if (CanHandle->Instance == FDCAN1) 
//   {
//     if (_CAN1->removeFromRingBuffer(_CAN1->txRing, txmsg))
//     {
//       _CAN1->write(txmsg, true);
//     }
//   }
// #ifdef CAN2
//   else if (CanHandle->Instance == CAN2) 
//   {
//     if (_CAN2->removeFromRingBuffer(_CAN2->txRing, txmsg))
//     {
//       _CAN2->write(txmsg, true);
//     }
//   }
// #endif
// #ifdef CAN3
//   else if (CanHandle->Instance == CAN3) 
//   {
//     if (_CAN3->removeFromRingBuffer(_CAN3->txRing, txmsg))
//     {
//       _CAN3->write(txmsg, true);
//     }
//   }
// #endif
// }

// extern "C" void HAL_CAN_TxMailbox2CompleteCallback( FDCAN_HandleTypeDef *CanHandle )
// {
//   CAN_message_t txmsg;
//   // use correct CAN instance
//   if (CanHandle->Instance == FDCAN1) 
//   {
//     if (_CAN1->removeFromRingBuffer(_CAN1->txRing, txmsg))
//     {
//       _CAN1->write(txmsg, true);
//     }
//   }
// #ifdef CAN2
//   else if (CanHandle->Instance == CAN2) 
//   {
//     if (_CAN2->removeFromRingBuffer(_CAN2->txRing, txmsg))
//     {
//       _CAN2->write(txmsg, true);
//     }
//   }
// #endif
// #ifdef CAN3
//   else if (CanHandle->Instance == CAN3) 
//   {
//     if (_CAN3->removeFromRingBuffer(_CAN3->txRing, txmsg))
//     {
//       _CAN3->write(txmsg, true);
//     }
//   }
// #endif
// }

// This is called by RX0_IRQHandler when there is message at RX FIFO0 buffer
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *CanHandle, uint32_t RxFifo0ITs)
{
  CAN_message_t rxmsg;
  FDCAN_RxHeaderTypeDef   RxHeader;
  //bool state = Disable_Interrupts();

  // move the message from RX FIFO0 to RX ringbuffer
  if (HAL_FDCAN_GetRxMessage( CanHandle, FDCAN_RX_FIFO0, &RxHeader, rxmsg.buf ) == HAL_OK)
  {
    rxmsg.id = RxHeader.Identifier;
    rxmsg.flags.extended = RxHeader.IdType == FDCAN_EXTENDED_ID;

    rxmsg.flags.remote = RxHeader.RxFrameType;
    // TODO: Unsure about this mb = FilterIndex
    rxmsg.mb           = RxHeader.FilterIndex;
    rxmsg.timestamp    = RxHeader.RxTimestamp;
    rxmsg.len          = RxHeader.DataLength;

    // use correct ring buffer based on CAN instance
    if (CanHandle->Instance == FDCAN1)
    {
      rxmsg.bus = 1;
      _CAN1->addToRingBuffer(_CAN1->rxRing, rxmsg);
    }
  }
  //Enable_Interrupts(state);
}

// RX IRQ handlers
extern "C" void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hcan1);
}

// // TX IRQ handlers
// extern "C" void FDCAN1_TX_IRQHandler(void)
// {
//   HAL_FDCAN_IRQHandler(&hcan1);
// }
