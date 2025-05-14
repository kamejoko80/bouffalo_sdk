# freertos

## Support CHIP

|      CHIP        | Remark |
|:----------------:|:------:|
|BL616             |        |

## Export PATH environment variable (on Windows git bash shell)

```
export PATH=$PATH:/c/msys64/mingw64/bin
export PATH=$PATH:$(realpath ../../tools/toolchains/bin)
```

## Compile

- CHIP
    + BL616
    + BL618

- MCU_MODULE
    + A
    + B

- FPGA_MODULE
    + GW1N_LV1
    + LMCXO2_256ZE
    + LMCXO2_640HC

```
make ninja CHIP=bl616 BOARD=bl616dk MCU_MODULE=A FPGA_MODULE=GW1N_LV1
```

## Flash

```
make flash CHIP=bl616 COMX=xxx # xxx is your com name
```