#include "init.h"
#include "gif-pros/gifclass.hpp"


void drawAutonSelector(){

	//Gif BOZO("/usd/[bozo].gif", lv_scr_act());
	Gif Bozo("/usd/[Bozo].gif", lv_scr_act());
	waitUntil(status.touch_status == 1);
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
				Gif RMA("/usd/[Red_Mogo_AWP].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				RMA.~Gif();
				break;
			}case 2:{
				//BMA
				Gif BMA("/usd/[Blue_Mogo_AWP].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				BMA.~Gif();
				break;
			}case 3:{
				//RRA
				Gif RRA("/usd/[Red_Ring_AWP].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				RRA.~Gif();
				break;
			}case 4:{
				//BRA
				Gif BRA("/usd/[Blue_Ring_AWP].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				BRA.~Gif();
				break;
			}case -1:{
				//RME
				Gif RME("/usd/[Red_Mogo_ELIM].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				RME.~Gif();
				break;
			}case -2:{
				//BME
				Gif BME("/usd/[Blue_Mogo_ELIM].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				BME.~Gif();
				break;
			}case -3:{
				//RRE
				Gif RRE("/usd/[Red_Ring_ELIM].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				RRE.~Gif();
				break;
			}case -4:{
				//BRE
				Gif BRE("/usd/[Blue_Ring_ELIM].gif", lv_scr_act());
				waitUntil(status.touch_status == 0);
				while(status.touch_status == 0){
					status = pros::screen::touch_status();
				}
				BRE.~Gif();
				break;
			}

		}
	}
}