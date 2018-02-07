/* 
 *
 *
 */
#include "tdoa_anc.h"

const uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

void tdoa_init(uint8 s1switch, dwt_config_t *config)
{
	dwt_setcallbacks(tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setsmarttxpower(1);

	dwt_setleds(1);
	dwt_setrxtimeout(RECEIVE_TIMEOUT);

	int anc_addr = (((s1switch & 0x10) << 2) + (s1switch & 0x20) + ((s1switch & 0x40) >> 2)) >> 4;
	ctx.anchorId = anc_addr;
	ctx.state = syncTdmaState;
	ctx.slot = NSLOTS-1;
	ctx.nextSlot = 0;
	ctx.msg_index = 0;
	ctx.A0_sync = 0;
	memset(ctx.rxTimestamps, 0, sizeof(ctx.rxTimestamps));
	memset(ctx.txTimestamps, 0, sizeof(ctx.txTimestamps));
	memset(ctx.distances, 0, sizeof(ctx.distances));
	memset(ctx.packetIds, 0, sizeof(ctx.packetIds));

	anc_prf = config->prf;
	anc_chan = config->chan;
}

void txKalman(dwTime_t *time)
{
	double xhat_temp[2];
	dwTime_t xout = { .full = 0 };

	xout.full = 0.495122939878641*ctx.xhat[0] + 0.504877060121359*(double)time->full;

	xhat_temp[0] = 0.490151653861331*ctx.xhat[0] + 0.002100512820513*ctx.xhat[1] + 0.509848346138669*(double)time->full;
	xhat_temp[1] = 2.366701106873675*((double)time->full - ctx.xhat[0]) + ctx.xhat[1];

	ctx.xhat[0] = (uint64_t)xhat_temp[0] & MASK_40BIT;
	ctx.xhat[1] = xhat_temp[1];

	if ((ctx.A0_sync == 0) && (abs(((int64_t)time->full - (int64_t)xout.full)) < KF_SYNC))
	{
		ctx.A0_sync = 1;
	}
	else if (ctx.A0_sync)
	{
		time->full = xout.full;
	}

}

void calculateDistance(uint8_t slot, uint8_t newId, uint32_t remoteTx, uint32_t remoteRx, uint32_t ts)
{
	// Check that the 2 last packets are consecutive packets
	if (ctx.packetIds[slot] == (newId-1))
	{
		int64_t tround1 = remoteRx - ctx.txTimestamps[ctx.slot];
		int64_t treply1 = ctx.txTimestamps[ctx.anchorId] - ctx.rxTimestamps[ctx.slot];
		int64_t tround2 = ts - ctx.txTimestamps[ctx.anchorId];
		int64_t treply2 = remoteTx - remoteRx;

		uint32_t distance = ((tround2 * tround1)-(treply1 * treply2)) / (2*(treply1 + tround2));
		ctx.distances[slot] = distance & 0xFFFFUL;
	}
	else
	{
		ctx.distances[slot] = 0;
	}
}

void setupTx()
{
	if(ctx.anchorId == 0) ctx.msg_index++;

	dwTime_t txTime = transmitTimeForSlot(ctx.nextSlot);
	ctx.txTimestamps[ctx.anchorId] = txTime.low32;

	setTxData();
	dwt_writetxfctrl(MAC802154_HEADER_LENGTH + sizeof(rangePacket_t) + FRAME_CRC, 0, 0);

	dwt_setdelayedtrxtime(txTime.high32);
	if(dwt_starttx(DWT_START_TX_DELAYED)) //delayed tx
	{
		//if the delayed TX failed then go back to listening
		dwt_setrxtimeout(RECEIVE_TIMEOUT); //reconfigure the timeout before enable
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

void setupRx()
{
	dwTime_t receiveTime = { .full = 0 };
	
	// Calculate start of the slot
	receiveTime.full = ctx.tdmaFrameStart.full + ctx.nextSlot*TDMA_SLOT_LEN;
	
	dwt_setrxtimeout(RECEIVE_TIMEOUT);

	dwt_setdelayedtrxtime(receiveTime.high32);
	if(dwt_rxenable(DWT_START_RX_DELAYED)) //delayed rx
	{
		//if the delayed RX failed - time has passed - do immediate enable
		dwt_setrxtimeout(RECEIVE_TIMEOUT); //reconfigure the timeout before enable
		//longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

void updateSlot()
{
	ctx.slot = ctx.nextSlot;
	ctx.nextSlot = ctx.nextSlot + 1;
	if(ctx.nextSlot >= NSLOTS) 
	{
		ctx.nextSlot = 0;
	}
	
	// If the next slot is 0, the next schedule has to be in the same frame!
	if(ctx.nextSlot == 0)
	{
		ctx.tdmaFrameStart.full += TDMA_FRAME_LEN;
	}
}

//#pragma GCC optimize ("O1")
void setTxData()
{
	static packet_t txPacket;
	static uint8 firstEntry = 1;
	
	if(firstEntry)
	{
		txPacket.fcf_s.type = 1;
		txPacket.fcf_s.security = 0;
		txPacket.fcf_s.framePending = 0;
		txPacket.fcf_s.ack = 0;
		txPacket.fcf_s.ipan = 1;
		txPacket.fcf_s.destAddrMode = 3;
		txPacket.fcf_s.version = 1;
		txPacket.fcf_s.srcAddrMode = 3;

		memcpy(txPacket.sourceAddress, base_address, 8);
		txPacket.sourceAddress[0] = ctx.anchorId;
		memcpy(txPacket.destAddress, base_address, 8);
		txPacket.destAddress[0] = 0xFF;
		
		txPacket.payload[0] = PACKET_TYPE_RANGE;
		
		firstEntry = 0;
	}
	
	rangePacket_t *rangePacket = (rangePacket_t *)txPacket.payload;
	
	rangePacket->idx = ctx.msg_index;
	for(int i=0; i<NSLOTS; i++)
	{
		memcpy(rangePacket->timestamps[i], &ctx.rxTimestamps[i], TS_TX_SIZE);
	}
	memcpy(rangePacket->timestamps[ctx.anchorId], &ctx.txTimestamps[ctx.anchorId], TS_TX_SIZE);
	memcpy(rangePacket->distances, ctx.distances, sizeof(ctx.distances));

	//dwSetData
	dwt_writetxdata(MAC802154_HEADER_LENGTH + sizeof(rangePacket_t) + FRAME_CRC, (uint8*)&txPacket, 0);
}

//#pragma GCC optimize ("O1")
dwTime_t transmitTimeForSlot(int slot)
{
	dwTime_t transmitTime = { .full = 0 };
	
	//calculate start of the slot
	transmitTime.full = ctx.tdmaFrameStart.full + slot*TDMA_SLOT_LEN;
	// Add guard and preamble time
	//transmitTime.full += TDMA_GUARD_LENGTH;
	//transmitTime.full += PREAMBLE_LENGTH;
	
	// DW1000 can only schedule time with 9 LSB at 0, adjust for it
	transmitTime.low32 = (transmitTime.low32 & ~((1<<9)-1)) + (1<<9);
	
	return transmitTime;
}

//#pragma GCC optimize ("O1")
void slotStep(const dwt_cb_data_t *cb_data, eventState_e event)
{
	switch (ctx.slotState) {
		case slotRxDone:
			if (event == RX_OK)
			{
				// start of handleRxPacket(dev)
				static packet_t rxPacket;
				dwTime_t rxTime = { .full = 0 };

				dwt_readrxtimestamp(rxTime.raw);
				dwCorrectTimestamp(&rxTime);

				dwt_readrxdata((uint8*)&rxPacket, cb_data->datalength, 0);

				if(cb_data->datalength == 0 || rxPacket.payload[0] != PACKET_TYPE_RANGE || rxPacket.sourceAddress[0] != ctx.slot)
				{
					//start of handleFailedRx
					ctx.rxTimestamps[ctx.slot] = 0;
					ctx.distances[ctx.slot] = 0;

					// Failed TDMA sync, keeps track of the number of fail so that the TDMA
					// watchdog can take decision as of TDMA resynchronisation
					if (ctx.slot == 0)
					{
						ctx.state = syncTdmaState;
					}
					//end of handleFailedRx
				}
				else
				{
					rangePacket_t * rangePacket = (rangePacket_t *)rxPacket.payload;

					uint32_t remoteTx, remoteRx;
					memcpy(&remoteTx, rangePacket->timestamps[ctx.slot], TS_TX_SIZE);
					memcpy(&remoteRx, rangePacket->timestamps[ctx.anchorId], TS_TX_SIZE);

					calculateDistance(ctx.slot, rangePacket->idx, remoteTx, remoteRx, rxTime.low32);

					ctx.packetIds[ctx.slot] = rangePacket->idx;
					ctx.rxTimestamps[ctx.slot] = rxTime.low32;
					memcpy(&ctx.txTimestamps[ctx.slot], &rangePacket->timestamps[ctx.slot], TS_TX_SIZE);

					// Resync and save useful anchor 0 information
					if(ctx.slot == 0)
					{
						//txKalman(&rxTime);

						//Resync local frame start to packet from anchor 0
						dwTime_t pkTxTime = { .full = 0 };
						memcpy(&pkTxTime, rangePacket->timestamps[ctx.slot], TS_TX_SIZE); //ctx.slot = 0
						ctx.tdmaFrameStart.full = rxTime.full;// - (pkTxTime.full - TDMA_LAST_FRAME(pkTxTime.full));

						ctx.msg_index = rangePacket->idx;
					}
				}
				// end of handleRxPacket
			}
			else
			{
				//start of handleFailedRx
				ctx.rxTimestamps[ctx.slot] = 0;
				ctx.distances[ctx.slot] = 0;

				// Failed TDMA sync, keeps track of the number of fail so that the TDMA
				// watchdog can take decision as of TDMA resynchronisation
				if (ctx.slot == 0)
				{
					ctx.state = syncTdmaState;
				}
				//end of handleFailedRx
			}

			// Quickly setup transfer to next slot
			if (ctx.nextSlot == ctx.anchorId)
			{
				setupTx();
				ctx.slotState = slotTxDone;
			}
			else
			{
				setupRx();
				ctx.slotState = slotRxDone;
			}

			break;
		case slotTxDone:
			// We send one packet per slot so after sending we setup the next receive
			setupRx();
			ctx.slotState = slotRxDone;
			break;
	}

	updateSlot();
}

void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	led_off(LED_ALL);
	led_on(LED_PC7);
	if(ctx.state == synchronizedState)
	{
		slotStep(cb_data, RX_OK);
	}
	else
	{
		if(ctx.anchorId == 0)
		{
			dwt_readsystime(ctx.tdmaFrameStart.raw);
			ctx.tdmaFrameStart.full = TDMA_LAST_FRAME(ctx.tdmaFrameStart.full) + 2*TDMA_FRAME_LEN;
			ctx.state = synchronizedState;
			setupTx();
			
			ctx.slotState = slotTxDone;
			updateSlot();
		}
		else
		{
			static packet_t rxPacket;
			dwTime_t rxTime = { .full = 0 };
			dwt_readrxtimestamp(rxTime.raw);
			dwCorrectTimestamp(&rxTime);
			dwt_readrxdata((uint8*)&rxPacket, cb_data->datalength, 0);
			
			if((rxPacket.sourceAddress[0] == 0) && (rxPacket.payload[0] == PACKET_TYPE_RANGE))
			{
				rangePacket_t * rangePacket = (rangePacket_t *)rxPacket.payload;

				//txKalman(&rxTime);

				dwTime_t pkTxTime = { .full = 0 };
				memcpy(&pkTxTime, rangePacket->timestamps[0], TS_TX_SIZE);
				ctx.tdmaFrameStart.full = rxTime.full;// - (pkTxTime.full - TDMA_LAST_FRAME(pkTxTime.full));
				
				ctx.tdmaFrameStart.full += TDMA_FRAME_LEN;
				
				ctx.msg_index = rangePacket->idx; //last sync index

				setupTx();

				ctx.slotState = slotRxDone;
				ctx.state = synchronizedState;
				updateSlot();
			}
			else
			{
				// Start the receiver waiting for a packet from anchor 0
				dwt_setrxtimeout(RECEIVE_TIMEOUT);
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
		}
	}
}

void rx_to_cb(const dwt_cb_data_t *cb_data)
{
	led_off(LED_ALL);
	led_on(LED_PC6);
	if(ctx.state == synchronizedState)
	{
		slotStep(cb_data, RX_TO);
	}
	else
	{
		if(ctx.anchorId == 0)
		{
			dwt_readsystime(ctx.tdmaFrameStart.raw);
			ctx.tdmaFrameStart.full = TDMA_LAST_FRAME(ctx.tdmaFrameStart.full) + 2*TDMA_FRAME_LEN;
			ctx.state = synchronizedState;
			setupTx();
			
			ctx.slotState = slotTxDone;
			updateSlot();
		}
		else
		{
			// Start the receiver waiting for a packet from anchor 0
			dwt_setrxtimeout(RECEIVE_TIMEOUT);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
	}
}

void rx_err_cb(const dwt_cb_data_t *cb_data)
{
	if(ctx.state == synchronizedState)
	{
		slotStep(cb_data, RX_ERR);
	}
	else
	{
		if(ctx.anchorId == 0)
		{
			dwt_readsystime(ctx.tdmaFrameStart.raw);
			ctx.tdmaFrameStart.full = TDMA_LAST_FRAME(ctx.tdmaFrameStart.full) + 2*TDMA_FRAME_LEN;
			ctx.state = synchronizedState;
			setupTx();
			
			ctx.slotState = slotTxDone;
			updateSlot();
		}
		else
		{
			// Start the receiver waiting for a packet from anchor 0
			dwt_setrxtimeout(RECEIVE_TIMEOUT);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
	}
}

void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
	led_off(LED_ALL);
	led_on(LED_PC9);
	if(ctx.state == synchronizedState)
	{
		slotStep(cb_data, TX_OK);
	}
	else
	{
		if(ctx.anchorId == 0)
		{
			dwt_readsystime(ctx.tdmaFrameStart.raw);
			ctx.tdmaFrameStart.full = TDMA_LAST_FRAME(ctx.tdmaFrameStart.full) + 2*TDMA_FRAME_LEN;
			ctx.state = synchronizedState;
			setupTx();
			
			ctx.slotState = slotTxDone;
			updateSlot();
		}
		else
		{
			// Start the receiver waiting for a packet from anchor 0
			dwt_setrxtimeout(RECEIVE_TIMEOUT);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
	}
}


static const uint8_t BIAS_500_16_ZERO = 10;
static const uint8_t BIAS_500_64_ZERO = 8;
static const uint8_t BIAS_900_16_ZERO = 7;
static const uint8_t BIAS_900_64_ZERO = 7;

// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
static const uint8_t BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31,   0,  36,  65,  84,  97, 106, 110, 112};
static const uint8_t BIAS_500_64[] = {110, 105, 100,  93,  82,  69,  51, 27,  0, 21,  35,  42,  49,  62,  71,  76,  81,  86};
static const uint8_t BIAS_900_16[] = {137, 122, 105, 88, 69,  47,  25,  0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
static const uint8_t BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};

void dwCorrectTimestamp(dwTime_t* timestamp)
{
	// base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
	float rxPowerBase = -(dwGetReceivePower() + 61.0f) * 0.5f;
	if (!isfinite(rxPowerBase)) {
		return;
	}
	int rxPowerBaseLow = (int)rxPowerBase;
	int rxPowerBaseHigh = rxPowerBaseLow + 1;
	if(rxPowerBaseLow < 0)
	{
		rxPowerBaseLow = 0;
		rxPowerBaseHigh = 0;
	}
	else if(rxPowerBaseHigh > 17)
	{
		rxPowerBaseLow = 17;
		rxPowerBaseHigh = 17;
	}
	// select range low/high values from corresponding table
	int rangeBiasHigh = 0;
	int rangeBiasLow = 0;
	if(anc_chan == 4 || anc_chan == 7)
	{
		// 900 MHz receiver bandwidth
		if(anc_prf == DWT_PRF_16M)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else if(anc_prf == DWT_PRF_64M)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else {
			// TODO proper error handling
		}
	}
	else
	{
		// 500 MHz receiver bandwidth
		if(anc_prf == DWT_PRF_16M)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
		}
		else if(anc_prf == DWT_PRF_64M)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
		}
		else
		{
			// TODO proper error handling
		}
	}
	// linear interpolation of bias values
	float rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
	// range bias [mm] to timestamp modification value conversion
	dwTime_t adjustmentTime;
//	float rangecalc = (rangeBias * DISTANCE_OF_RADIO_INV * 0.001f);
//	int rangecheck = (int)rangecalc;
	adjustmentTime.full = (int)(rangeBias * DISTANCE_OF_RADIO_INV * 0.001f);
	// apply correction
	timestamp->full += adjustmentTime.full;
}

float dwGetReceivePower(void)
{
	uint8 rxFrameInfo[RX_FINFO_LEN];
	float C = (float)dwt_read16bitoffsetreg(RX_FQUAL_ID, CIR_PWR_OFFSET);
	dwt_readfromdevice(RX_FINFO_ID,RX_FINFO_OFFSET,RX_FINFO_LEN,rxFrameInfo);
	float N = (float)((((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4));

	return calculatePower(C * TWOPOWER17, N);
}

float calculatePower(float base, float N)
{
	float A, corrFac;

	if(DWT_PRF_16M == anc_prf)
	{
		A = 115.72f;
		corrFac = 2.3334f;
	}
	else
	{
		A = 121.74f;
		corrFac = 1.1667f;
	}

	float estFpPwr = 10.0f * log10f(base / (N * N)) - A;

	if(estFpPwr <= -88)
	{
		return estFpPwr;
	}
	else
	{
		// approximation of Fig. 22 in user manual for dbm correction
		estFpPwr += (estFpPwr + 88) * corrFac;
	}

	return estFpPwr;
}
