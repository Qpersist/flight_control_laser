SET PATH=E:\Keil_v5\ARM\ARMCC\Bin;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;D:\USERPROFILE%\AppData\Local\Microsoft\WindowsApps
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F407ZE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Anopilot
SET CPU_CLOCK=0x00B71B00
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\main.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\stm32f4xx_it.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\scheduler.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\init.__i"
"E:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\build\parameter.__i"
