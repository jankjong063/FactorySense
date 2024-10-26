@echo off
setlocal

:: Set the source path for the .elf file (adjust the path if necessary)
set "SOURCE_PATH=C:\Users\88691\AppData\Local\Temp\arduino\sketches\FEEBCEFB46BD198FA1B8DB970293C4B8\FactorySense.ino.elf"

:: Set the destination folder (where the .ino file is located)
set "DEST_FOLDER=F:\PHD\Arduino_Projects\FactorySense\FactorySense_v15.0"

:: Path to avr-objcopy tool (adjust the version if necessary)
set "AVR_OBJCOPY=C:\Users\88691\AppData\Local\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin\avr-objcopy.exe"

:: Copy the .elf file to the destination folder
if exist "%SOURCE_PATH%" (
    copy "%SOURCE_PATH%" "%DEST_FOLDER%\FactorySense.ino.elf"
    echo .elf file copied to %DEST_FOLDER%
    
    :: Convert .elf to .bin using the full path to avr-objcopy
    "%AVR_OBJCOPY%" -O binary "%DEST_FOLDER%\FactorySense.ino.elf" "%DEST_FOLDER%\FactorySense.ino.bin"
    echo .bin file generated at %DEST_FOLDER%
    
    :: Convert .elf to .hex using the full path to avr-objcopy
    "%AVR_OBJCOPY%" -O ihex "%DEST_FOLDER%\FactorySense.ino.elf" "%DEST_FOLDER%\FactorySense.ino.hex"
    echo .hex file generated at %DEST_FOLDER%
) else (
    echo Source file does not exist: "%SOURCE_PATH%"
)

endlocal
