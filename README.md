# ti-poe
Linux port of TI TPS23880 firmware upload and monitor code

The PoE controller is programmable. You can control many aspects of its
operations.

Use this command to build the 'main_auto' binary:
```
${CROSS}-gcc -Wall -Os -I. -o main_auto *.c
```
***Note:***
We tested gcc version 10 here. It should work with older versions as
well.
