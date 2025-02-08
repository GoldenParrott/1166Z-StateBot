#include "init.h"
#include "gif-pros/gifclass.hpp"

void drawAutonSelector(){

	Bozo._render();
	waitUntil(status.touch_status == 1)
	status = pros::screen::touch_status();
	Bozo.~Gif();
	while(1){
		// Used when the AWP/ELIM button is pressed
		if ((status.x >= 253)&&(status.x <= 328)&&(status.y >= 127)&&(status.y <= 230)){
		
			// Switches positives to negatives 
			switch(autonnumber){
				case 1:{
					autonnumber = -1;
					break;
				}case 2:{
					autonnumber = -2;
					break;
				}case 3:{
					autonnumber = -3;
					break;
				}case 4:{
					autonnumber = -4;
					break;
				}case -1:{
					autonnumber = 1;
					break;
				}case -2:{
					autonnumber = 2;
					break;
				}case -3:{
					autonnumber = 3;
					break;
				}case -4:{
					autonnumber = 4;
					break;
				}
			}
		}else if((status.x > 0)&&(status.x < 120)&&(status.y > 120)&&(status.y < 240)){
		// Used when bottom left of the field is pressed 
			autonnumber = 1;
		}else if((status.x > 120)&&(status.x < 240)&&(status.y > 120)&&(status.y < 240)){
		// Used when bottom right of the field is pressed 
			autonnumber = 2;
		}else if((status.x > 0)&&(status.x < 120)&&(status.y > 0)&&(status.y < 120)){
		// Used when top left of the field is pressed 
			autonnumber = 3;
		}else if((status.x > 120)&&(status.x < 240)&&(status.y > 0)&&(status.y < 120)){
		// Used when bottom right of the field is pressed 
			autonnumber = 4;
		}
		
		switch (autonnumber){
			case 1:{
				//RMA
				RMA._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RMA.~Gif();
				break;
			}case 2:{
				//BMA
				BMA._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BMA.~Gif();
				break;
			}case 3:{
				//RRA
				RRA._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RRA.~Gif();
				break;
			}case 4:{
				//BRA
				BRA._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BRA.~Gif();
				break;
			}case -1:{
				//RME
				RME._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RME.~Gif();
				break;
			}case -2:{
				//BME
				BME._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BME.~Gif();
				break;
			}case -3:{
				//RRE
				RRE._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RRE.~Gif();
				break;
			}case -4:{
				//BRE
				BRE._render();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BRE.~Gif();
				break;
			}

		}
	}
}