# CC1352 Hacks

## CC1352 RF Radio

- Dump Binary: cc1352_M0_rom.hex

### Chipwhisperer app
The app will handle simpleserial2 commands from chipwisperer and send a TX frame with the given address.
The main core go in sleep just after sending the command to the doorbell in order to avoid voltage glitch impact on it.
The redirection of RFC_DBELL_SYSGPOCTL_GPOCTL0_MCEGPO0 on a GPIO, allowed with help of logical analyzer to dump the data when the glitchs succeed
/!\ Very messy code, provided as it is /!\

This app permits with the help of a chipwhisperer to dump the M0 rom (and discover simpler way to dump it after some analysis...)

## Goal ?
- Try to understand how set a custom frequency over all the range 169-1000 Mhz
