Files :
i2c_flash.c : 			This is for non blocking implementation of code.
i2c_flash_block.c : 	This is for blocking implementation of code.
main_2.c : 				This is the User code to test both the implementation. 
Makefile :				Make file to create the objects

----------------------------------------------------------------------------------------
Steps to Follow :
1. Load the .ko object in the board using "insmod"
2. Run the user code can be run using ./main_2

To test Write:
- Run "./main_2 write 'noOfPages'" 
- eg "./main_2 write 1" - to read one page
- minimum noOfPages is One.
- All the pages are filled with 'a'

To test Read:
- Run "./main_2 read 'noOfPages' "
- eg. "./main_2 read 1" to read one page
- If read returns a "Try Again" Please try the same command again
- minimum noOfPages is One.

To test Flash Erase:
- Run "./main_2 flasherase"
- Please wait for some time after the command.

To test FlashGetp:
- Run "./main_2 flashgetp"
- This returns back the current Read/Write pointer 

To test FlashSetp:
- Run "./main_2 flashsetp 'pageNumber'"
- e.g. "./main_2 flashsetp 0" this moves to the Zero'th page. 
- pageNumber is between 0 and 511.

To test FlashGets:
- Run "./main_2 flashgets"
- This displays the status of EEPROM

--------------------------------------------------------------------------------
References:
1. linux source code i2c_dev.c
