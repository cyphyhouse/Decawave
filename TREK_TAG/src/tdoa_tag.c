/* 
 *
 *
 */
#include "tdoa_tag.h"


uint32_t statsReceivedPackets = 0;
uint32_t statsAcceptedAnchorDataPackets = 0;
uint32_t statsAcceptedPackets = 0;

//float uwbTdoaDistDiff[NR_OF_ANCHORS][256];
//uint8_t num = 0;

uint8 previousAnchor;
rangePacket_t rxPacketBuffer[NR_OF_ANCHORS];
dwTime_t arrivals[NR_OF_ANCHORS];
static uint8_t sequenceNrs[NR_OF_ANCHORS];

double clockCorrection_T_To_A[NR_OF_ANCHORS];

void tdoa_init(uint8 s1switch, dwt_config_t *config)
{
	dwt_setcallbacks(NULL, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	dwt_setsmarttxpower(1);

	dwt_setrxtimeout(10000);

	//memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));
	previousAnchor = 0;

	anc_prf = config->prf;
	anc_chan = config->chan;
	usbDataReady = 0;
}

static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
	return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
	return fullTimeStamp & 0x00FFFFFFFFul;
}

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
static uint8 isValidTimeStamp(const int64_t anchorRxTime) {
	return anchorRxTime != 0;
}

static uint8 isSeqNrConsecutive(uint8_t prevSeqNr, uint8_t currentSeqNr) {
	return (currentSeqNr == ((prevSeqNr + 1) & 0xFF));
}

static uint8 calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
	if (! isSeqNrConsecutive(rxPacketBuffer[anchor].Idx, packet->Idx)) {
		return 0;
	}

	const int64_t previous_txAn_in_cl_An = rxPacketBuffer[anchor].timestamps[anchor];
	const int64_t rxAn_by_T_in_cl_T = arrival->full;
	const int64_t txAn_in_cl_An = packet->timestamps[anchor];
	const int64_t previous_rxAn_by_T_in_cl_T = arrivals[anchor].full;
	const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
	const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);

	*clockCorrection = frameTime_in_cl_An / frameTime_in_T;
	return 1;
}

static uint8 calcDistanceDiff(float* tdoaDistDiff, const uint8_t previousAnchor, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
	const int64_t rxAn_by_T_in_cl_T  = arrival->full;
	const int64_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];
	const int64_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
	const double clockCorrection = clockCorrection_T_To_A[anchor];

	const uint8 isSeqNrInAnchorOk = isSeqNrConsecutive(sequenceNrs[anchor], packet->Idx);
	const uint8 isAnchorDistanceOk = isValidTimeStamp(tof_Ar_to_An_in_cl_An);
	const uint8 isRxTimeInTagOk = isValidTimeStamp(rxAr_by_An_in_cl_An);
	const uint8 isClockCorrectionOk = (clockCorrection != 0.0);

	if (! (isSeqNrInAnchorOk && isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
		return 0;
	}

	const int64_t txAn_in_cl_An = packet->timestamps[anchor];
	const int64_t rxAr_by_T_in_cl_T = arrivals[previousAnchor].full;

	const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
	const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection - delta_txAr_to_txAn_in_cl_An;

	*tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;

	return 1;
}

#pragma GCC optimize ("O3")
void rx_ok_cb(const dwt_cb_data_t *rxd)
{
	statsReceivedPackets++;

	packet_t rxPacket;

	dwTime_t arrival = {.full = 0};
	dwt_readrxtimestamp(arrival.raw);
	dwCorrectTimestamp(&arrival);

	dwt_readrxdata((uint8 *)&rxPacket, rxd->datalength, 0);  // Read Data Frame

	const uint8_t anchor = rxPacket.sourceAddress[0] & 0xFF;

	if (anchor < NR_OF_ANCHORS)
	{
		const rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

		calcClockCorrection(&clockCorrection_T_To_A[anchor], anchor, packet, &arrival);

		if (anchor != previousAnchor)
		{
			float tdoaDistDiff = 0.0f;

			if (calcDistanceDiff(&tdoaDistDiff, previousAnchor, anchor, packet, &arrival))
			{
				statsAcceptedAnchorDataPackets++;

				usbData.distanceDiff = tdoaDistDiff;
				usbData.prevAnc = previousAnchor;
				usbData.currAnc = anchor;
				usbDataReady = 1;

			}
		}

		arrivals[anchor].full = arrival.full;
		memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));
		sequenceNrs[anchor] = packet->Idx;

		previousAnchor = anchor;
	}

	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_to_cb(const dwt_cb_data_t *cb_data)
{
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_err_cb(const dwt_cb_data_t *cb_data)
{
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
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

#pragma GCC optimize ("O3")
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
	adjustmentTime.full = (int)(rangeBias * DISTANCE_OF_RADIO_INV * 0.001f);
	// apply correction
	timestamp->full += adjustmentTime.full;
}

#pragma GCC optimize ("O3")
float dwGetReceivePower(void)
{
	uint8 rxFrameInfo[RX_FINFO_LEN];
	float C = (float)dwt_read16bitoffsetreg(RX_FQUAL_ID, CIR_PWR_OFFSET);
	dwt_readfromdevice(RX_FINFO_ID,RX_FINFO_OFFSET,RX_FINFO_LEN,rxFrameInfo);
	float N = (float)((((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4));

	return calculatePower(C * TWOPOWER17, N);
}

#pragma GCC optimize ("O3")
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
