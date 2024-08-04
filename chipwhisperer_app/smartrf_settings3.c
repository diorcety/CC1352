//*********************************************************************************
// Generated by SmartRF Studio version 2.30.0 (build#397)
// The applied template is compatible with cc13x2_26x2 SDK version 2.30.xx.xx or newer.
// Device: CC1352P Rev. E (2.1). 
//
//*********************************************************************************


//*********************************************************************************
// Parameter summary
// RX Address0: 0xAA 
// RX Address1: 0xBB 
// RX Address Mode: No address check 
// Frequency: 224.70001 MHz
// Data Format: Serial mode disable 
// Deviation: 2.406 kHz
// Fixed Packet Length: 64 
// Packet Length Config: Fixed 
// Max Packet Length: 255 
// Packet Data: 255 
// Preamble Count: 4 Bytes 
// Preamble Mode: Send 0 as the first preamble bit 
// RX Filter BW: 8.5 kHz
// Symbol Rate: 2.39999 kBaud
// Sync Word: 0x0000f68d 
// Sync Word Length: 16 Bits 
// TX Power: 13 dBm 
// Whitening: No whitening 

#include "smartrf_settings.h"

#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_prop.h)

// TI-RTOS RF Mode Object
RF_Mode RF_prop =
{
    .rfMode = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_prop,
    .mcePatchFxn = 0,
    .rfePatchFxn = 0
};


// Overrides for CMD_PROP_RADIO_DIV_SETUP_PA
uint32_t pOverrides[] =
{
    // override_prop_common.xml
    // DC/DC regulator: In Tx with 14 dBm PA setting, use DCDCCTL5[3:0]=0xF (DITHER_EN=1 and IPEAK=7). In Rx, use default settings.
    (uint32_t)0x00F788D3,
    // override_prop_common_sub1g.xml
    // Set RF_FSCA.ANADIV.DIV_SEL_BIAS = 1. Bits [0:16, 24, 30] are don't care..
    (uint32_t)0x4001405D,
    // Set RF_FSCA.ANADIV.DIV_SEL_BIAS = 1. Bits [0:16, 24, 30] are don't care..
    (uint32_t)0x08141131,
    // override_tc220_tc221_ch1-5_tx.xml
    // Synth: Set loop bandwidth after lock to 75 kHz with damping = 1.1 (K2)
    (uint32_t)0x777C0583,
    // Synth: Set loop bandwidth after lock to 75 kHz with damping = 1.1 (K2)
    (uint32_t)0x000005A3,
    // Synth: Set loop bandwidth after lock to 75 kHz (K3, LSB)
    (uint32_t)0xCC320603,
    // Synth: Set loop bandwidth after lock to 75 kHz (K3, MSB)
    (uint32_t)0x00010623,
    // Synth: Set FREF = 6.857 (48 MHz/7) MHz
    (uint32_t)0x000784A3,
    // Tx: Configure PA ramp time, PACTL2.RC=0x1 (in ADI0, set PACTL2[4:3]=0x1)
    ADI_2HALFREG_OVERRIDE(0,16,0x8,0x8,17,0x1,0x1),
    // Tx: Set wait time before turning off ramp to 0x1A (default: 0x1F)
    HW_REG_OVERRIDE(0x6028,0x001A),
    // Tx: Set repetition factor = 2
    HW_REG_OVERRIDE(0x5324,0x0002),
    (uint32_t)0xFFFFFFFF
};


// CMD_PROP_RADIO_DIV_SETUP_PA
// Proprietary Mode Radio Setup Command for All Frequency Bands
rfc_CMD_PROP_RADIO_DIV_SETUP_PA_t RF_cmdPropRadioDivSetup =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x5,
    .modulation.deviation = 0x9A,
    .modulation.deviationStepSz = 0x2,
    .symbolRate.preScale = 0xFC,
    .symbolRate.rateWord = 0xCE70,
    .symbolRate.decimMode = 0x0,
    .rxBw = 0x44,
    .preamConf.nPreamBytes = 0x4,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x10,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x1,
    .formatConf.fecMode = 0x2,
    .formatConf.whitenMode = 0x0,
    .config.frontEndMode = 0x0,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .config.bSynthNarrowBand = 0x1,
    .txPower = 0x003F,
    .pRegOverride = pOverrides,
    .centerFreq = 0x00E0,
    .intFreq = 0x8000,
    .loDivider = 0x1E,
    .pRegOverrideTxStd = 0,
    .pRegOverrideTx20 = 0
};


// CMD_FS
// Frequency Synthesizer Programming Command
rfc_CMD_FS_t RF_cmdFs =
{
    .commandNo = 0x0803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .frequency = 0x00E0,
    .fractFreq = 0xB334,
    .synthConf.bTxMode = 0x1,
    .synthConf.refFreq = 0x0,
    .__dummy0 = 0x00,
    .__dummy1 = 0x00,
    .__dummy2 = 0x00,
    .__dummy3 = 0x0000
};


// CMD_PROP_TX
// Proprietary Mode Transmit Command
rfc_CMD_PROP_TX_t RF_cmdPropTx =
{
    .commandNo = 0x3801,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bUseCrc = 0x1,
    .pktConf.bVarLen = 0x0,
    .pktLen = 0x40,
    .syncWord = 0x0000F68D,
    .pPkt = 0 // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

