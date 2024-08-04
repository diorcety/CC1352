__attribute__ ((section (".TI.ramfunc"), target("thumb"))) uint32_t read_value(uint32_t *arg) {
    *arg = *(uint32_t*)(0x4);
    return 0;
}

struct rfc_CMD_USER_FUN_s RF_user_Fun = {
    .commandNo = 0x0811,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .userFunAddr = (uint32_t)read_value,
    .pParams = 0
};

RF_runCmd(rfHandle, (RF_Op*)&RF_user_Fun, RF_PriorityNormal, NULL, RF_EventLastCmdDone);
