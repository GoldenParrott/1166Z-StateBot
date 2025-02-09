#include "init.h"
#include "gif-pros/gifclass.hpp"

void drawAutonSelector(){

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
	waitUntil(status.touch_status == 1)
	status = pros::screen::touch_status();
	Bozo.pause();
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
	}
}

void drawBasicSelector(){
	while(1){

		//Sets the Updated Screen
		if(autonnumber == 0){
			pros::screen::set_eraser(0x00FF0000);
		}else{
			pros::screen::set_eraser(0x00000000);
		}
		// Reset All Drawings
		pros::screen::erase_rect(0,0,480,240);
	
		// Field Selector
		pros::screen::set_pen(0x009400D3);
		switch(autonnumber){
			case 1:{ // Blue Mogo
				pros::screen::fill_rect(120,120,243,240);
				break;
			}
			case 2:{ // Blue Ring
				pros::screen::fill_rect(120,0,243,120);
				break;
			}
			case -1:{ // Red Mogo
				pros::screen::fill_rect(0,120,120,240);
				break;
			}
			case -2:{ // Red Ring
				pros::screen::fill_rect(0,0,120,120);
				break;
			}
		}
	
		switch(globalAuton){
			case true:{
				pros::screen::fill_rect(243,0,480,120);
				break;
			}
			case false:{
				pros::screen::fill_rect(243,120,480,240);
				break;
			}
		}
	
		// Field Grid
		pros::screen::set_pen(0x00FFFFFF);
		pros::screen::fill_rect(118,240,122,0);
		pros::screen::fill_rect(240,240,244,0);
		pros::screen::fill_rect(0,118,480,122);
	
	
		// Words
			// Field
		pros::screen::set_pen(0x00FF0000);
		pros::screen::print(TEXT_MEDIUM_CENTER,18,53,"RED RING",NULL);
		pros::screen::print(TEXT_MEDIUM_CENTER,18,173,"RED MOGO",NULL);
		pros::screen::set_pen(0x000000FF);
		pros::screen::print(TEXT_MEDIUM_CENTER,138,53,"BLUE RING",NULL);
		pros::screen::print(TEXT_MEDIUM_CENTER,138,173,"BLUE MOGO",NULL);
			// Global Switch
		pros::screen::set_pen(0x00FFFFFF);
		if(autonnumber == 2){
			pros::screen::print(TEXT_MEDIUM_CENTER,315,53,"SAFETY",NULL);
		}else{
			pros::screen::print(TEXT_MEDIUM_CENTER,315,53,"UNIVERSAL",NULL);
		}
		
		pros::screen::print(TEXT_MEDIUM_CENTER,330,173,"UNIQUE",NULL);
	
		/* Guides
		pros::screen::set_pen(COLOR_WHITE);
		pros::screen::draw_line(60,0,60,240);
		pros::screen::draw_line(183,0,183,240);
		pros::screen::draw_line(0,60,480,60);
		pros::screen::draw_line(0,180,480,180);
		/**/
	
		// Waits For User Input and Updates Autonnumber
		while(status.touch_status != 0){
			status = pros::screen::touch_status();
		}
		while(status.touch_status != 1){
			status = pros::screen::touch_status();
		}
	
		// Used when the AWP/ELIM button is pressed
		if (status.x >= 245){
		
			if(status.y > 120){
				globalAuton = false;
			}else if(status.y < 120){
				globalAuton = true;
			}
	
		}else if((status.x >= 0)&&(status.x <= 120)&&(status.y >= 120)&&(status.y <= 244)){
		// Used when bottom left of the field is pressed 
			autonnumber = -1;
		}else if((status.x >= 120)&&(status.x <= 244)&&(status.y >= 120)&&(status.y <= 244)){
		// Used when bottom right of the field is pressed 
			autonnumber = 1;
		}else if((status.x >= 0)&&(status.x <= 120)&&(status.y >= 0)&&(status.y <= 120)){
		// Used when top left of the field is pressed 
			autonnumber = -2;
		}else if((status.x >= 120)&&(status.x <= 244)&&(status.y >= 0)&&(status.y <= 120)){
		// Used when bottom right of the field is pressed 
			autonnumber = 2;
		}
		pros::delay(10);
	}
}
