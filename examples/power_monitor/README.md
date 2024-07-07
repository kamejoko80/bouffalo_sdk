# power_monitor


## Support CHIP

|      CHIP        | Remark |
|:----------------:|:------:|
|BL616             |        |

## Compile

- BL616/BL618

```
make ninja CHIP=bl616 BOARD=bl616dk
```

## Flash

```
make flash CHIP=bl616 COMX=xxx # xxx is your com name
```