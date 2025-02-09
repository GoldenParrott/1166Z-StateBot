#include "init.h"
#include "gif-pros/gifclass.hpp"

void drawAutonSelector(){
	/*
	Bozo.pause();
	Logo.pause();
	RMA.pause();
	BMA.pause();
	RRA.pause();
	BRA.pause();
	RME.pause();
	BME.pause();
	RRE.pause();
	BRE.pause();

	Bozo.resume();
	*/
	waitUntil(status.touch_status == 1)
	status = pros::screen::touch_status();
	// Bozo.pause();
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
		/*
		switch (autonnumber){
			case 1:{
				//RMA
				RMA.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RMA.pause();
				break;
			}case 2:{
				//BMA
				BMA.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BMA.pause();
				break;
			}case 3:{
				//RRA
				RRA.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RRA.pause();
				break;
			}case 4:{
				//BRA
				BRA.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BRA.pause();
				break;
			}case -1:{
				//RME
				RME.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RME.pause();
				break;
			}case -2:{
				//BME
				BME.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BME.pause();
				break;
			}case -3:{
				//RRE
				RRE.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				RRE.pause();
				break;
			}case -4:{
				//BRE
				BRE.resume();
				waitUntil(status.touch_status == 0);
				waitUntil(status.touch_status == 1)
				BRE.pause();
				break;
			}

		}
		*/
	}
}

void drawBasicSelector(){
	
}