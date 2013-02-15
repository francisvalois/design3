#include "commandes.h"
#include "ecran.h"
#include <string.h>



char commandeDecode(char position, char opcode, char  arg)
{
 switch(opcode) 
	{
        case LED:
           if (arg == 0x00)
              GPIO_PORTD_DATA_R |= GPIO_PIN_0;
           else if (arg == 0x01)
              GPIO_PORTD_DATA_R &= ~(GPIO_PIN_0);
           break;
        case CLRLINE:
           ecranClear();
           ecranWriteLine("VC DF",5);
           ecranSetPosCursor(0x40);
           position=0;
           break;
        case WRITE:
        	if(position <16)
        	{
        		ecranWriteChar(arg);
        		position++;
       		}
           break;                                  
        default:
           break; 
    }
    return position;
}
