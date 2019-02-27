# Tutorial

## Source
http://regalis.com.pl/en/arm-cortex-stm32-gnulinux/
https://blog.jetbrains.com/clion/2016/06/clion-for-embedded-development/
https://os.mbed.com/platforms/ST-Nucleo-F302R8/
https://github.com/libopencm3/libopencm3/blob/master/lib/stm32/common/spi_common_all.c

## Asjad mida ma õppisin (väga väsitava) eksperimenteerimise käigus
Väga kasulikud on manualid, spekid ja lisaks näidiskood (internetist inimeste tehtud).
Ise tuleb katsetada kõiki hüpoteese sest kui otse low-level kirjutada (ilma hal-ita) on
suhteliselt vähe ressursse internetis. Kõik teevad teekidega vms.

- Port F pinid ei tööta by default. Miks? Ei tea! Kusagil kirjas pole.

- SPI puhul peavad STM32 ja teine osapool olema GND-iga ühendatud. Muidu on suhtlusprobleemid.

- SPI3 ja muidu SPI2 töötavad väga kergelt, aga GPIO conf on vaja õigeks saada.

- SPI transfer töötab iseenesest - tuleb lihtsalt DR registrisse vajalik byte kirjutada ja
masin teeb ise transferi.

## Kompileri määramine
Clionis - mine preferences->build-exec->CMake ja pane profiilile 
"CMake options"-isse -D CMAKE_CXX_COMPILER=<> ja -D CMAKE_C_COMPILER=<>

1.
STMF32 Cube download (leidsin mingi lambise githubi)

2.
Copyda vajalikud failid projekti ümber. Need on nüüd toetusfailid mingi
spetsiifilise micro jaoks et me saaks sinna proge teha.

Minimaalsed failid on:
    stm32fxx.h
    stm32f<prose_nimi>.h
    system_stm32f3xx.h
    kogu Drivers/CMSIS/Include sisu
    # See on nüüd esimene fail mis nö bootib micro
    startup_stm32f302x8.s

Lisaks VÕIB copyda system_stm32<prose_nimi>.c faili mis confib alustamisel prose,
aga võib ka ise kirjutada.

Veidi kergemaks teevad proge HAL failid, ehk high-level teek. Samas kui tahta
õppida siis võiks neid välistada. Need leiab
Drivers/STM32F3xx_HAL_Driver kaustast

3.
GNU GCC ARM kompailer
Repodes täitsa olemas.

sudo apt-get install arm-none-eabi-gcc libnewlib-arm-none-eabi

4.
Minimaalne programm

Kaks faili on vajalikud
system.c - meie implementatsioon system_stm32fxx.h-st mis peab micro bootimisel konfima
main.c - main() funktsioon ja kasutaja kood

5.
Kompileerimine

NB! Kui CMake hakkab hädaldama et linker faili pole siis copy terve scripts/ kaust
cmake-build-debug/CMakeFiles/CMakeTmp alla

Kõigepealt C kood
arm-none-eabi-gcc -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb -Ilibrary -DSTM32F302x8 -Os -c <faili_nimi>.h <faili_nimi>.c

Siis linkimine
arm-none-eabi-gcc -mcpu=cortex-m4 -mlittle-endian -mthumb -DSTM32F302x8 ~/Projects/STM32CubeF3-master/Projects/STM32F302R8-Nucleo/Templates/SW4STM32/STM32F302R8-Nucleo/STM32F302R8Tx_FLASH.ld -Wl,--gc-sections --specs=nosys.specs system.o main.o startup.o -o main.elf

Lõpuks .hex fail
arm-none-eabi-objcopy -Oihex main.elf main.hex

6.
Flashimine

NB! MCU peab nyyd usbiga arvutisse ühendatud olema

Openocd kui pole installitud
sudo apt-get install openocd

Ühendame nucleoga läbi openocd
openocd -f /usr/share/openocd/scripts/board/st_nucleo_f3.cfg

Ühendame openocd-ga läbi telneti
telnet localhost 4444

(Telnet-is) flashime prose ära
reset halt
flash write_image erase <hexfile_nimi>.hex
reset run
