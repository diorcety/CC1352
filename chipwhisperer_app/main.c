/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include "ti/devices/cc13x2_cc26x2/driverlib/gpio.h"
#include "ti/devices/cc13x2_cc26x2/driverlib/interrupt.h"
#include <sys/_stdint.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/rfc.h>
#include "ti/drivers/rf/RFCC26X2.h"
#include <sys/cdefs.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <assert.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_ble_mailbox.h)

/* Driver configuration */
#include "ti_drivers_config.h"
#include "ti_radio_config.h"
#include "smartrf_settings.h"
#include "simpleserial.h"

/* Packet TX Configuration */
#define PACKET_DATA_LENGTH   27 /* Packet for BLE5 cannot exceed 37 with header (10) */
#define PACKET_INTERVAL      RF_convertMsToRatTicks(100) /* Set packet interval to 500ms */

/***** Variable declarations *****/
static RF_Object rfBleObject;
static RF_Handle rfBleHandle;

static RF_Object rfPropObject;
static RF_Handle rfPropHandle;

static UART2_Handle uartHandle;

static Timer_Handle timerHandle;

static uint8_t packet[PACKET_DATA_LENGTH];
static uint16_t seqNumber;

void rfDriverCallbackAntennaSwitching(RF_Handle client, RF_GlobalEvent events, void *arg)
{
#if 1
    if (events & RF_GlobalEventRadioSetup)
    {
        bool    sub1GHz   = false;
        uint8_t loDivider = 0;

        GPIO_write(CONFIG_GPIO_ENABLE_RX, GPIO_CFG_OUT_HIGH);
        GPIO_write(CONFIG_GPIO_ENABLE_TX, GPIO_CFG_OUT_HIGH);

        /* Decode the current PA configuration. */
        RF_TxPowerTable_PAType paType = (RF_TxPowerTable_PAType)RF_getTxPower(client).paType;

        /* Decode the generic argument as a setup command. */
        RF_RadioSetup* setupCommand = (RF_RadioSetup*)arg;

        switch (setupCommand->common.commandNo) {
            case (CMD_RADIO_SETUP):
            case (CMD_BLE5_RADIO_SETUP):
                    loDivider = RF_LODIVIDER_MASK & setupCommand->common.loDivider;

                    /* Sub-1GHz front-end. */
                    if (loDivider != 0) {
                        sub1GHz = true;
                    }
                    break;
            case (CMD_PROP_RADIO_DIV_SETUP):
                    loDivider = RF_LODIVIDER_MASK & setupCommand->prop_div.loDivider;

                    /* Sub-1GHz front-end. */
                    if (loDivider != 0) {
                        sub1GHz = true;
                    }
                    break;
            default:break;
        }

        if (sub1GHz) {
            /* Sub-1 GHz */
            if (paType == RF_TxPowerTable_HighPA) {
                GPIO_write(CONFIG_GPIO_ENABLE_RX, GPIO_CFG_OUT_HIGH);
                GPIO_write(CONFIG_GPIO_ENABLE_TX, GPIO_CFG_OUT_LOW);
            } else {
                GPIO_write(CONFIG_GPIO_ENABLE_RX, GPIO_CFG_OUT_LOW);
                GPIO_write(CONFIG_GPIO_ENABLE_TX, GPIO_CFG_OUT_HIGH);
            }
        }
    }
    else if (events & RF_GlobalEventRadioPowerDown)
    {
        GPIO_write(CONFIG_GPIO_ENABLE_RX, GPIO_CFG_OUT_HIGH);
        GPIO_write(CONFIG_GPIO_ENABLE_TX, GPIO_CFG_OUT_HIGH);
    }
#endif
}

/***** Function definitions *****/
static void txDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventLastCmdDone)
    {
        /* Successful TX */
        if (h == rfPropHandle)
        {
            //GPIO_toggle(CONFIG_GPIO_LED_0);
        }
    }
}

static bool sleep_enable = false;
volatile bool new_interrupt = false;
volatile uint32_t new_pOp = 0;
static void RF_hwiHw(uintptr_t a)
{
    RFCAckIntClear();
    new_interrupt = true;
}


#define SLEEP_EN

uint32_t
RFCDoorbellSendTo(uint32_t pOp)
{
    IntDisable(INT_UART0_COMB);
    IntDisable(INT_GPT0A);

    // Wait until the doorbell becomes available
    while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) != 0);
    RFCAckIntClear();
    new_interrupt = false;

    //GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);
#ifdef SLEEP_EN
    if (sleep_enable)
    {
        //HwiP_enableInterrupt(INT_RFC_CMD_ACK);
        new_pOp = pOp;
    }
    else
    {
    // Submit the command to the CM0 through the doorbell
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = pOp;
    }
#else
    // Submit the command to the CM0 through the doorbell
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = pOp;
#endif


#ifdef SLEEP_EN
    if (sleep_enable)
    {
        /*
        __asm("wfi");
        while(new_interrupt == false);
        new_interrupt = false;
        HwiP_disableInterrupt(INT_RFC_CMD_ACK);
        */
    }
    else
    {
        // Wait until the CM0 starts to parse the command
        while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
        RFCAckIntClear();
        new_interrupt = false;
    }
#else
    // Wait until the CM0 starts to parse the command
    while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
    RFCAckIntClear();
    new_interrupt = false;
#endif

    //GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_OFF);

    // Return with the content of status register
    uint32_t v = (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA));


    IntEnable(INT_UART0_COMB);
    IntEnable(INT_GPT0A);

    return v;
}

static void aa()
{
    IntDisable(INT_UART0_COMB);
    IntDisable(INT_GPT0A);

    __asm("wfi");

    IntEnable(INT_UART0_COMB);
    IntEnable(INT_GPT0A);
}

void timerCallback(Timer_Handle handle, int_fast16_t status)
{
    GPIO_toggle(CONFIG_GPIO_LED_0);
}

void __attribute__ ((noinline)) delay(int i)
{
    while(i-- > 0) __asm("nop");
}

volatile unsigned int send_msg = 0;

char getch(void)
{
    char data;
    size_t read;
    UART2_read(uartHandle, &data, 1, &read);
    return data;
}

void putch(char c)
{
    size_t written;
    UART2_write(uartHandle, &c, 1, &written);
}

#define REPLY(S) simpleserial_put('r', sizeof(S), (uint8_t*)(S))
#define ERROR(S) simpleserial_put('e', sizeof(S), (uint8_t*)(S))

uint8_t sleep_cmd(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data)
{
    REPLY("SLEEP");
    aa();
    return 0;
}

uint8_t a_cmd(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data)
{
    REPLY("AAA");
    return 0;
}


uint8_t send_message_cmd(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data)
{
    uint32_t cmdStatus;
    uint32_t start_time;
    bool good = false;
    RF_EventMask terminationReason;

    if (dlen == 8)
    {
        RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
        RF_cmdPropTx.pPkt = *(uint8_t**)(&data[0]);
        RF_cmdPropTx.pktLen = *(unsigned int *)(&data[4]);

        start_time = RF_getCurrentTime();
        sleep_enable = true;
        //GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);
        RF_CmdHandle ch = RF_postCmd(rfPropHandle, (RF_Op*) &RF_cmdPropTx, RF_PriorityNormal, txDoneCallback, RF_EventLastCmdDone);
        while(ch == RF_ALLOC_ERROR)
        {
            GPIO_toggle(CONFIG_GPIO_LED_0);
            usleep(500000);
            GPIO_toggle(CONFIG_GPIO_LED_0);
            usleep(500000);
        }
        sleep_enable = false;
        terminationReason = RF_EventLastCmdDone;

        while(new_pOp == 0);
        if (new_pOp != 0)
        {
            // Wait until the doorbell becomes available
            while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) != 0);
            RFCAckIntClear();

            IntDisable(INT_UART0_COMB);
            IntDisable(INT_GPT0A);

            GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);

            // Submit the command to the CM0 through the doorbell
            HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = new_pOp;

            __asm("wfi");

            GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_OFF);

            // Wait until the CM0 starts to parse the command
            while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
            RFCAckIntClear();

            IntEnable(INT_UART0_COMB);
            IntEnable(INT_GPT0A);

            new_pOp = 0;
        }

        while (!(RF_pendCmd(rfPropHandle, ch, RF_EventLastCmdDone) & RF_EventLastCmdDone))
        {
            if (RF_getCurrentTime() - start_time > RF_convertMsToRatTicks(500))
            {
                terminationReason = RF_EventCmdCancelled;
                RF_flushCmd(rfPropHandle, ch, 0);
                break;
            }
        }

        //GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_OFF);

        if(terminationReason & RF_EventCmdPreempted)
        {
            // It is possible for a scheduled command to be preempted by another
            // higher priority command. In this case the RF driver will either
            // cancel/abort/stop the preempted command and return the appropriate
            // event flag. Additionally, the command preempted event flag is also set.

        }

        // Mask off the RF_EventCmdPreempted bit to allow further processing
        // in the switch-case block
        switch (terminationReason & ~(RF_EventCmdPreempted))
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
#if 1
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
                // by RF_cancelCmd() or RF_flushCmd().
                good = false;
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                good = false;
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                good = false;
                break;
#endif
            default:
                // Uncaught error event
                while (1)
                {
                    GPIO_toggle(CONFIG_GPIO_LED_0);
                    usleep(500000);
                    GPIO_toggle(CONFIG_GPIO_LED_0);
                    usleep(500000);
                }
        }

        cmdStatus = ((volatile RF_Op*) &RF_cmdPropTx)->status;

        switch (cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
#if 1
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                good = false;
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                good = false;
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                good = false;
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_BLE_RADIO_SETUP or CMD_RADIO_SETUP
                good = false;
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                good = false;
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                good = false;
                break;
#endif
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while (1)
                {
                    GPIO_toggle(CONFIG_GPIO_LED_0);
                    usleep(500000);
                    GPIO_toggle(CONFIG_GPIO_LED_0);
                    usleep(500000);
                }
        }

        simpleserial_put('r', sizeof(cmdStatus), (uint8_t*)&cmdStatus);
    }
    else
    {
        ERROR("INV");
    }
    return 0;
}

/* RF core HW hardware interrupts */
static HwiP_Struct RF_hwiHwObj;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    GPIO_setConfigAndMux(CONFIG_GPIO_0, GPIO_CFG_OUTPUT | GPIO_CFG_INPUT, IOC_PORT_RFC_GPO0);

    /* Turn on user LED */
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
#if 0
    GPIO_write(CONFIG_GPIO_ENABLE_RX, GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_ENABLE_TX, GPIO_CFG_OUT_LOW);
#endif
#if 1
    /* Initialize multimode scheduling params
     * - Params shared between rf drivers since commands are synchronous
     * - Ignore end time
     * - Priority should not affect transmission
     */
    RF_ScheduleCmdParams schParams;
    schParams.endTime = 0;
    // TODO: Need to replace with updated parameters from RFLIB-107
    // schParams.priority = RF_PriorityNormal;
    schParams.allowDelay = RF_AllowDelayAny;
    schParams.activityInfo = 0;
    schParams.duration = 0;
    schParams.endType = RF_EndNotSpecified;
    schParams.startTime = 0;
    schParams.startType = RF_StartNotSpecified;

    /* Initialize HWI used by the RF driver. */
    HwiP_Params hp;
    HwiP_Params_init(&hp);
    hp.priority = 0;
    HwiP_construct(&RF_hwiHwObj,   INT_RFC_CMD_ACK, RF_hwiHw,           &hp);
    HwiP_disableInterrupt(INT_RFC_CMD_ACK);

    /* Initialize Prop RF Driver */
    RF_Params rfPropParams;
    RF_Params_init(&rfPropParams);

    /* Set mode for multiple clients */
    RF_prop.rfMode = RF_MODE_MULTIPLE;

    /* Request access to the prop radio and
     * - Radio is not powered on by RF_open
     * - RF_cmdFs will power on the radio to cache the frequency settings
     */
    rfPropHandle = RF_open(&rfPropObject, &RF_prop,
                           (RF_RadioSetup*) &RF_cmdPropRadioDivSetup,
                           &rfPropParams);
    assert(rfPropHandle != NULL);
    (void)RF_runCmd(rfPropHandle, (RF_Op*) &RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Configure timer */
    Timer_Params    params;
    Timer_Params_init(&params);
    params.periodUnits = Timer_PERIOD_HZ;
    params.period = 2;
    params.timerMode  = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timerHandle = Timer_open(CONFIG_TIMER_0, &params);
    assert(timerHandle != NULL);
    //assert(Timer_start(timerHandle) == Timer_STATUS_SUCCESS);

    /* Initialize UART with callback read mode */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 230400;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartHandle = UART2_open(CONFIG_UART2_0, &uartParams);
    assert(uartHandle != NULL);

    /* Configure simpleserial */
    simpleserial_init();
    simpleserial_addcmd('b', 8, send_message_cmd);
    simpleserial_addcmd('a', 8, a_cmd);
    simpleserial_addcmd('s', 0, sleep_cmd);

    while (1)
    {
        //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_OFF);

        while(1)
        {
            simpleserial_get();
            //REPLY("CMD");
        }
    }
#endif
}