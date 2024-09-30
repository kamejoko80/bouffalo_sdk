# gowin_fpga_config

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

- BL616/BL618

```
make ninja CHIP=bl616 BOARD=bl616dk MCU_MODULE=A (or MCU_MODULE=B)
```

## Flash

```
make flash CHIP=bl616 COMX=xxx # xxx is your com name
```